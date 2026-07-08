#include "pose_server.hh"

#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"

#include "pose_protocol_generated.h"

#include <App.h>         // uWS
#include <libusockets.h> // us_create_timer / us_timer_set / us_timer_ext

#include <spdlog/spdlog.h>

#include <atomic>
#include <charconv>
#include <mutex>
#include <bit>

namespace net
{
    namespace fb = flatbuffers;
    namespace fb_proto = exo::proto;

    namespace
    {
        // windows.h (pulled in by uWebSockets) defines a GetMessage macro that shadows
        // the generated flatbuffers GetMessage() accessor; isolate the workaround here.
        const fb_proto::Message* get_root_message(const uint8_t* data)
        {
#pragma push_macro("GetMessage")
#undef GetMessage
            return fb_proto::GetMessage(data);
#pragma pop_macro("GetMessage")
        }
    }

    // --- observer (worker thread) ------------------------------------------------
    // Worker-thread tag detection; latches the detections for the server loop to pull.
    class pose_frame_observer final : public hw::sensor_frame_observer
    {
    public:
        pose_frame_observer(const hw::sensor_frame_provider& provider, double tag_size_m)
            : _provider{ provider }, _tag_size_m{ tag_size_m }
        { }

        // Returns false if nothing new since `last_seq`, else copies out + advances it.
        bool try_get(
            std::vector<pose::tag_detection_t>& out_dets,
            std::chrono::microseconds& out_timestamp,
            uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_dets = _detections;
            out_timestamp = _timestamp;
            last_seq = _seq;
            return true;
        }

        // True once per stream-end signal (consumes the latched flag).
        bool consume_stream_ended_signal() noexcept {
            return _stream_ended.exchange(false);
        }

    public:
        void on_sensor_frame_update(const std::shared_ptr<hw::sensor_frame>& frame) override
        {
            if (!_detector.has_value()) // built once intrinsics are known (after open)
            {
                pose::tag_detector::options_t opt;
                opt.intrinsics = _provider.get_calibration().color_intr;
                opt.tag_size_m = _tag_size_m;
                _detector.emplace(opt);
            }

            auto detections = _detector.value().detect(frame->color_image);

            std::scoped_lock lk{ _mtx };
            _detections = std::move(detections);
            _timestamp = frame->timestamp;
            ++_seq;
        }

        void on_sensor_stream_reset() override {}
        void on_sensor_stream_end() override { _stream_ended = true; }

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        std::optional<pose::tag_detector> _detector; // built once intrinsics are known (after open)
        std::mutex _mtx;
        std::vector<pose::tag_detection_t> _detections;
        std::chrono::microseconds _timestamp{ 0 }; // device timestamp of the latched frame
        uint64_t _seq{ 0 };
        std::atomic<bool> _stream_ended{ false }; // set by the worker thread on stream end
    };

    // --- pose_server -------------------------------------------------------------
    pose_server::pose_server(
        uint16_t port, 
        const app::source_options& initial)
        : _port{ port }
        , _initial{ initial }
    { }

    bool pose_server::_do_open_source_stream(
        const std::string& source, 
        double tag_size_m,
        std::optional<int32_t> exposure_us, 
        std::optional<int32_t> gain)
    {
        auto provider = std::make_shared<hw::sensor_frame_provider>();
        auto observer = std::make_shared<pose_frame_observer>(*provider, tag_size_m);
        provider->add_observer(observer);

        // Parse: a full unsigned integer is a device index, anything else a path.
        uint32_t device_index{};
        const auto [ptr, ec] = std::from_chars(source.data(), source.data() + source.size(), device_index);
        const bool is_device = (ec == std::errc{} && ptr == source.data() + source.size());

        const bool ok = is_device
            ? provider->open_device(device_index, exposure_us, gain)
            : provider->open_recording(source);
        if (!ok)
        {
            spdlog::error("failed to open source '{}'", source);
            return false;
        }

        _provider = std::move(provider); // old provider closes/joins here
        _observer = std::move(observer);
        _is_recording = !is_device;
        _last_seq = 0;
        _estimator.clear_rest_pose();
        spdlog::info("source '{}' opened", _provider->get_source_name());
        return true;
    }

    void pose_server::_do_close_source_stream()
    {
        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _is_recording = false;
        _last_seq = 0;
    }

    bool pose_server::_do_calibrate_rest_pose()
    {
        return _estimator.calibrate_rest_pose();
    }

    void pose_server::_do_clear_rest_pose()
    {
        _estimator.clear_rest_pose();
    }

    bool pose_server::_poll_new_detections()
    {
        if (!_observer || !_observer->try_get(_detections, _last_timestamp, _last_seq)) { return false; }
        _estimator.update(_detections, _last_timestamp);
        return true;
    }

    bool pose_server::_poll_stream_ended()
    {
        return _observer && _observer->consume_stream_ended_signal();
    }

    std::string pose_server::_serialize_pose_frame() const
    {
        fb::FlatBufferBuilder b;

        std::vector<fb::Offset<fb_proto::JointPose>> joints;
        joints.reserve(pose::kNumJoints);
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _estimator.get_joint_state(info.id);

            const auto to_fb_quat = [](const Eigen::Quaterniond& q) {
                return fb_proto::Quat{ q.x(), q.y(), q.z(), q.w() };
            };

            // NOTE: fb_local_rot, fb_local_anim_rot must outlive CreateJointPose, which copies them immediately.
            fb_proto::Quat fb_local_rot, fb_local_anim_rot;
            if (st.local_rot.has_value()) { fb_local_rot = to_fb_quat(st.local_rot.value()); }
            if (st.local_anim_rot.has_value()) { fb_local_anim_rot = to_fb_quat(st.local_anim_rot.value()); }

            // Optional rotations: write a Quat only when present, else nullptr. (absent on the wire)
            joints.push_back(fb_proto::CreateJointPose(
                b, static_cast<fb_proto::JointId>(info.id),
                st.local_rot.has_value() ? &fb_local_rot : nullptr,
                st.local_anim_rot.has_value() ? &fb_local_anim_rot : nullptr
            ));
        }

        const auto joints_vec = b.CreateVector(joints);
        const uint32_t frame_id = _provider ? _provider->get_current_frame_id() : 0;
        const auto pose_frame = fb_proto::CreatePoseFrame(b, frame_id, _last_timestamp.count(), _estimator.has_rest_pose(), joints_vec);

        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_PoseFrame, pose_frame.Union(), kServerNotifyReqId));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string pose_server::_serialize_server_status(req_id_t req_id) const
    {
        fb::FlatBufferBuilder b;

        const bool opened = static_cast<bool>(_provider);
        const auto name = b.CreateString(opened ? _provider->get_source_name() : std::string{});
        int32_t w = 0, h = 0;
        if (opened)
        {
            const auto res = _provider->get_color_camera_resolution();
            w = res.x(); h = res.y();
        }

        const auto status = fb_proto::CreateServerStatus(
            b, opened, name, w, h, _estimator.has_rest_pose()
        );

        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_ServerStatus, status.Union(), req_id));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string pose_server::_serialize_source_stream_ended() const
    {
        fb::FlatBufferBuilder b;
        // Recording EOF is graceful; a live device stopping on its own is a loss.
        const bool is_error = !_is_recording;
        const char* msg = _is_recording ? "recording reached end" : "device stream ended";
        const auto ended = fb_proto::CreateSourceStreamEnded(b, is_error, b.CreateString(msg));
        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_SourceStreamEnded, ended.Union(), kServerNotifyReqId));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string pose_server::_serialize_ack(
        bool ok, 
        std::string_view msg, 
        req_id_t req_id) const
    {
        fb::FlatBufferBuilder b;
        const auto ack = fb_proto::CreateAck(b, ok, b.CreateString(msg.data(), msg.size()));
        b.Finish(fb_proto::CreateMessage(b, fb_proto::Payload_Ack, ack.Union(), req_id));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    int pose_server::run()
    {
        struct socket_data {
            bool accepted{ false }; // false for a rejected (over-capacity) socket
        };

        uWS::App app;

        app.ws<socket_data>("/*", {
            .compression = uWS::DISABLED,
            .open = [this](auto* ws) {
                if (_client_count >= 1) {
                    ws->end(1013, "another client is already connected"); // 1013 = Try Again Later
                    return;
                }
                ws->getUserData()->accepted = true;
                ++_client_count;
                // Subscribe to the broadcast topics, then send the current status.
                ws->subscribe("pose");
                ws->subscribe("status");
                ws->send(this->_serialize_server_status(), uWS::OpCode::BINARY);
            },
            .message = [this, &app](auto* ws, std::string_view msg, uWS::OpCode /*op*/) {
                if (!ws->getUserData()->accepted) { return; } // ignore a rejected socket still closing
                const auto* data = std::bit_cast<const uint8_t*>(msg.data());
                fb::Verifier verifier{ data, msg.size() };
                if (!fb_proto::VerifyMessageBuffer(verifier))
                {
                    ws->send(this->_serialize_ack(false, "malformed message"), uWS::OpCode::BINARY);
                    return;
                }

                const fb_proto::Message* m = get_root_message(data);
                const req_id_t req = m->request_id(); // echoed back on the reply for correlation

                // A malformed request must never take down the loop; reply with an error Ack.
                try
                {
                    if (req == kServerNotifyReqId) {
                        // 0 is reserved for server notify events,
                        // so a client command carrying request id 0 is a protocol violation.
                        // reject it.
                        ws->send(this->_serialize_ack(false, "request_id must be non-zero", kServerNotifyReqId), uWS::OpCode::BINARY);
                        return;
                    }

                    // Pure query(no Ack, no broadcast):
                    // reply with the current status to this client only.
                    if (m->payload_type() == fb_proto::Payload_GetServerStatus)
                    {
                        ws->send(this->_serialize_server_status(req), uWS::OpCode::BINARY);
                        return;
                    }

                    bool status_changed = false;
                    std::string ack;

                    switch (m->payload_type())
                    {
                        case fb_proto::Payload_OpenSourceStream:
                        {
                            const auto* o = m->payload_as_OpenSourceStream();
                            const std::string src = o->source() ? o->source()->str() : std::string{};
                            std::optional<int32_t> exposure, gain;
                            if (const auto e = o->exposure_us()) { exposure = *e; }
                            if (const auto g = o->gain()) { gain = *g; }
                            const bool ok = this->_do_open_source_stream(src, o->tag_size_m(), exposure, gain);
                            ack = this->_serialize_ack(ok, ok ? "source opened" : "open failed", req);
                            status_changed = true;
                            break;
                        }
                        case fb_proto::Payload_CloseSourceStream:
                            this->_do_close_source_stream();
                            ack = this->_serialize_ack(true, "source closed", req);
                            status_changed = true;
                            break;
                        case fb_proto::Payload_CalibrateRestPose:
                        {
                            const bool ok = this->_do_calibrate_rest_pose();
                            ack = this->_serialize_ack(ok, ok ? "rest pose calibrated" : "no computable joint rotation", req);
                            status_changed = true;
                            break;
                        }
                        case fb_proto::Payload_ClearRestPose:
                            this->_do_clear_rest_pose();
                            ack = this->_serialize_ack(true, "rest pose cleared", req);
                            status_changed = true;
                            break;
                        default:
                            ack = this->_serialize_ack(false, "unsupported message", req);
                            break;
                    }

                    ws->send(ack, uWS::OpCode::BINARY);
                    if (status_changed) { app.publish("status", this->_serialize_server_status(), uWS::OpCode::BINARY); }
                }
                catch (const std::exception& e)
                {
                    spdlog::error("command handler error: {}", e.what());
                    ws->send(this->_serialize_ack(false, "internal error", req), uWS::OpCode::BINARY);
                }
            },
            .close = [this](auto* ws, int /*code*/, std::string_view /*message*/) {
                if (!ws->getUserData()->accepted) { return; } // rejected socket was never counted
                // Release the shared source once the (only) client disconnects.
                --_client_count;
                if (_client_count == 0)
                {
                    this->_do_close_source_stream();
                    spdlog::info("last client disconnected; source released");
                }
            }
        });

        // Periodic pump on the loop thread: pull the latest detections, update the
        // estimator, and broadcast a PoseFrame. The provider's worker thread only
        // latches, so this is the single place pose data is serialized and sent.
        struct timer_ctx { pose_server* self; uWS::App* app; };
        auto* timer = ::us_create_timer(std::bit_cast<us_loop_t*>(uWS::Loop::get()), 0, sizeof(timer_ctx));
        auto* ctx = std::bit_cast<timer_ctx*>(::us_timer_ext(timer));
        ctx->self = this;
        ctx->app = &app;
        ::us_timer_set(timer, [](us_timer_t* t) {
            auto* c = std::bit_cast<timer_ctx*>(::us_timer_ext(t));
            if (c->self->_poll_new_detections())
            {
                c->app->publish("pose", c->self->_serialize_pose_frame(), uWS::OpCode::BINARY);
            }
            if (c->self->_poll_stream_ended())
            {
                c->app->publish("status", c->self->_serialize_source_stream_ended(), uWS::OpCode::BINARY);
            }
        }, 8, 8); // ~120 Hz

        // Optional: auto-open a recording passed on the command line.
        if (!_initial.input_path.empty())
        {
            this->_do_open_source_stream(
                _initial.input_path, 
                _initial.tag_size_m,
                _initial.exposure_us, 
                _initial.gain
            );
        }

        app.listen(_port, [this](auto* token) {
            if (token) { spdlog::info("pose server listening on ws://localhost:{}", _port); }
            else { spdlog::error("failed to listen on port {}", _port); }
        }).run();

        return 0;
    }

} // namespace net
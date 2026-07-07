#include "pose_server.hh"

#include "hw/sensor_frame_observer.hh"
#include "pose/tag_detector.hh"

#include "pose_protocol_generated.h"

#include <App.h>         // uWebSockets
#include <libusockets.h> // us_create_timer / us_timer_set / us_timer_ext

#include <spdlog/spdlog.h>

#include <charconv>
#include <mutex>
#include <bit>

// uWebSockets pulls in <windows.h>, whose GetMessage macro clobbers the generated
// flatbuffers GetMessage() accessor. Undo it for this translation unit.
#ifdef GetMessage
#undef GetMessage
#endif

namespace net
{
    namespace fb = flatbuffers;
    using namespace exo::proto;

    // --- observer (worker thread) ------------------------------------------------
    // Worker-thread tag detection; latches the detections for the server loop to pull.
    class pose_frame_observer final : public hw::sensor_frame_observer
    {
    public:
        pose_frame_observer(const hw::sensor_frame_provider& provider, double tag_size_m)
            : _provider{ provider }, _tag_size_m{ tag_size_m }
        { }

        // Returns false if nothing new since `last_seq`, else copies out + advances it.
        bool try_get(std::vector<pose::tag_detection_t>& out_dets, uint64_t& last_seq)
        {
            std::scoped_lock lk{ _mtx };
            if (_seq == last_seq) { return false; }
            out_dets = _detections;
            last_seq = _seq;
            return true;
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
            ++_seq;
        }

        void on_sensor_stream_reset() override {}
        void on_sensor_stream_end() override {}

    private:
        const hw::sensor_frame_provider& _provider;
        double _tag_size_m{};
        std::optional<pose::tag_detector> _detector; // built once intrinsics are known (after open)
        std::mutex _mtx;
        std::vector<pose::tag_detection_t> _detections;
        uint64_t _seq{ 0 };
    };

    // --- pose_server -------------------------------------------------------------
    pose_server::pose_server(
        uint16_t port, 
        const app::source_options& initial)
        : _port{ port }
        , _initial{ initial }
    { }

    bool pose_server::_apply_open(const std::string& source, double tag_size_m,
                                  std::optional<int32_t> exposure_us, std::optional<int32_t> gain)
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

    void pose_server::_apply_close()
    {
        _provider.reset(); // stops/joins the worker thread
        _observer.reset();
        _is_recording = false;
        _last_seq = 0;
    }

    bool pose_server::_apply_calibrate()
    {
        return _estimator.calibrate_rest_pose();
    }

    void pose_server::_apply_clear_rest()
    {
        _estimator.clear_rest_pose();
    }

    bool pose_server::_poll_new_detections()
    {
        if (!_observer || !_observer->try_get(_detections, _last_seq)) { return false; }
        _estimator.update(_detections);
        return true;
    }

    std::string pose_server::_serialize_pose_frame() const
    {
        fb::FlatBufferBuilder b;

        std::vector<fb::Offset<JointPose>> joints;
        joints.reserve(pose::kNumJoints);
        for (const auto& info : pose::kJointsInfo)
        {
            const auto& st = _estimator.get_joint_state(info.id);
            const Eigen::Quaterniond ql = st.local_rot.value_or(Eigen::Quaterniond::Identity());
            const Eigen::Quaterniond qa = st.local_anim_rot.value_or(Eigen::Quaterniond::Identity());
            const Quat local_rot{ ql.x(), ql.y(), ql.z(), ql.w() };
            const Quat local_anim_rot{ qa.x(), qa.y(), qa.z(), qa.w() };
            joints.push_back(CreateJointPose(
                b, static_cast<JointId>(info.id), st.is_visible(), &local_rot, &local_anim_rot));
        }

        const auto joints_vec = b.CreateVector(joints);
        const uint32_t frame_id = _provider ? _provider->get_current_frame_id() : 0;
        const auto pose_frame = CreatePoseFrame(b, frame_id, 0, _estimator.has_rest_pose(), joints_vec);

        b.Finish(CreateMessage(b, Payload_PoseFrame, pose_frame.Union()));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string pose_server::_serialize_server_status() const
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

        const auto status = CreateServerStatus(
            b, opened, name, w, h, _is_recording, _estimator.has_rest_pose()
        );

        b.Finish(CreateMessage(b, Payload_ServerStatus, status.Union()));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    std::string pose_server::_serialize_ack(bool ok, std::string_view message) const
    {
        fb::FlatBufferBuilder b;
        const auto ack = CreateAck(b, ok, b.CreateString(message.data(), message.size()));
        b.Finish(CreateMessage(b, Payload_Ack, ack.Union()));
        return std::string(std::bit_cast<const char*>(b.GetBufferPointer()), b.GetSize());
    }

    int pose_server::run()
    {
        struct socket_data {}; // per-connection state (unused)

        uWS::App app;

        app.ws<socket_data>("/*", {
            .compression = uWS::DISABLED,
            .open = [this](auto* ws) {
                // Subscribe to the broadcast topics, then send the current status.
                ++_client_count;
                ws->subscribe("pose");
                ws->subscribe("status");
                ws->send(this->_serialize_server_status(), uWS::OpCode::BINARY);
            },
            .message = [this, &app](auto* ws, std::string_view msg, uWS::OpCode /*op*/) {
                const auto* data = std::bit_cast<const uint8_t*>(msg.data());
                fb::Verifier verifier{ data, msg.size() };
                if (!VerifyMessageBuffer(verifier))
                {
                    ws->send(this->_serialize_ack(false, "malformed message"), uWS::OpCode::BINARY);
                    return;
                }

                const Message* m = GetMessage(data);

                // A malformed request must never take down the loop; reply with an error Ack.
                try
                {
                    // Pure query(no Ack, no broadcast):
                    // reply with the current status to this client only.
                    if (m->payload_type() == Payload_GetServerStatus)
                    {
                        ws->send(this->_serialize_server_status(), uWS::OpCode::BINARY);
                        return;
                    }

                    bool status_changed = false;
                    std::string ack;

                    switch (m->payload_type())
                    {
                        case Payload_OpenSource:
                        {
                            const auto* o = m->payload_as_OpenSource();
                            const std::string src = o->source() ? o->source()->str() : std::string{};
                            std::optional<int32_t> exposure, gain;
                            if (const auto e = o->exposure_us()) { exposure = *e; }
                            if (const auto g = o->gain()) { gain = *g; }
                            const bool ok = this->_apply_open(src, o->tag_size_m(), exposure, gain);
                            ack = this->_serialize_ack(ok, ok ? "source opened" : "open failed");
                            status_changed = true;
                            break;
                        }
                        case Payload_CloseSource:
                            this->_apply_close();
                            ack = this->_serialize_ack(true, "source closed");
                            status_changed = true;
                            break;
                        case Payload_CalibrateRestPose:
                        {
                            const bool ok = this->_apply_calibrate();
                            ack = this->_serialize_ack(ok, ok ? "rest pose calibrated" : "no computable joint rotation");
                            status_changed = true;
                            break;
                        }
                        case Payload_ClearRestPose:
                            this->_apply_clear_rest();
                            ack = this->_serialize_ack(true, "rest pose cleared");
                            status_changed = true;
                            break;
                        default:
                            ack = this->_serialize_ack(false, "unsupported message");
                            break;
                    }

                    ws->send(ack, uWS::OpCode::BINARY);
                    if (status_changed) { app.publish("status", this->_serialize_server_status(), uWS::OpCode::BINARY); }
                }
                catch (const std::exception& e)
                {
                    spdlog::error("command handler error: {}", e.what());
                    ws->send(this->_serialize_ack(false, "internal error"), uWS::OpCode::BINARY);
                }
            },
            .close = [this](auto* /*ws*/, int /*code*/, std::string_view /*message*/) {
                // Release the shared source once the last client disconnects.
                if (_client_count > 0) { --_client_count; }
                if (_client_count == 0)
                {
                    this->_apply_close();
                    spdlog::info("last client disconnected; source released");
                }
            }
        });

        // Periodic pump on the loop thread: pull the latest detections, update the
        // estimator, and broadcast a PoseFrame. The provider's worker thread only
        // latches, so this is the single place pose data is serialized and sent.
        struct timer_ctx { pose_server* self; uWS::App* app; };
        auto* timer = ::us_create_timer(std::bit_cast<us_loop_t*>(uWS::Loop::get()), 0, sizeof(timer_ctx));
        auto* ctx = std::bit_cast<timer_ctx*>(us_timer_ext(timer));
        ctx->self = this;
        ctx->app = &app;
        ::us_timer_set(timer, [](us_timer_t* t) {
            auto* c = std::bit_cast<timer_ctx*>(::us_timer_ext(t));
            if (c->self->_poll_new_detections())
            {
                c->app->publish("pose", c->self->_serialize_pose_frame(), uWS::OpCode::BINARY);
            }
        }, 8, 8); // ~120 Hz

        // Optional: auto-open a recording passed on the command line.
        if (!_initial.input_path.empty())
        {
            this->_apply_open(_initial.input_path, _initial.tag_size_m, _initial.exposure_us, _initial.gain);
        }

        app.listen(_port, [this](auto* token) {
            if (token) { spdlog::info("pose server listening on ws://localhost:{}", _port); }
            else { spdlog::error("failed to listen on port {}", _port); }
        }).run();

        return 0;
    }

} // namespace net

#pragma once
#include "cli_options.hh"

#include "hw/sensor_frame_provider.hh"
#include "pose/pose_estimator.hh"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace net
{
    // forward declaration of worker-thread observer
    class pose_frame_observer;

    // Protocol correlation id (Message.request_id).
    using req_id_t = uint32_t;

    // request_id value reserved for messages the server sends on its own
    // (status/pose/ended notifications); clients must use a non-zero id.
    inline constexpr req_id_t kServerNotifyReqId{ 0 };

    // uWS based pose server. (protocol: pose_protocol.fbs)
    //
    // NOTE: uWS runs one event loop on the calling thread.
    //       The provider's worker thread only latches detections;
    //       everything else happens on the loop thread, 
    //       so no extra locking is needed here.
    //       (command dispatch, estimator update, all sends ...)
    class pose_server final
    {
    public:
        pose_server(uint16_t port, const app::source_options& initial);

        pose_server(const pose_server&) = delete;
        pose_server& operator=(const pose_server&) = delete;
        pose_server(pose_server&&) = delete;
        pose_server& operator=(pose_server&&) = delete;

        // blocks: listen + run the event loop
        int run();

    private:
        // Pipeline ops (loop thread)
        bool _do_open_source_stream(
            const std::string& source, // unsigned int: device index / else: recording path.
            double tag_size_m,
            std::optional<int32_t> exposure_us, 
            std::optional<int32_t> gain
        );
        void _do_close_source_stream();
        bool _do_calibrate_rest_pose();
        void _do_clear_rest_pose();

        // Pull latched detections and recompute joint states;
        // true if a new frame arrived.
        bool _poll_new_detections();

        // True once per stream-end signal latched by the observer (worker thread).
        bool _poll_stream_ended();

        // Protocol serializers (loop thread) -> FlatBuffers `Message` bytes.
        // Pass req_id to echo the triggering command's id back on a reply;
        // NOTE: 0 is reserved for messages the server sends on its own.
        std::string _serialize_pose_frame() const;
        std::string _serialize_server_status(req_id_t req_id = kServerNotifyReqId) const;
        std::string _serialize_source_stream_ended() const;
        std::string _serialize_ack(
            bool ok, 
            std::string_view msg, 
            req_id_t req_id = kServerNotifyReqId
        ) const;

    private:
        uint16_t _port;
        app::source_options _initial;

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;
        pose::pose_estimator _estimator;

        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };
        std::chrono::microseconds _last_timestamp{ 0 }; // device timestamp of the latched frame
        bool _is_recording{ false };
        size_t _client_count{ 0 }; // connected clients; source is released when it hits 0
    };

} // namespace net

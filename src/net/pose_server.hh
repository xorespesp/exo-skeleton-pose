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

    // uWS based pose server. (protocol: pose_protocol.fbs)
    //
    // NOTE: uWS runs one event loop on the calling thread.
    //       The provider's worker thread only latches detections;
    //       everything else (command dispatch, estimator update, all sends) happens on the loop thread, 
    //       so no extra locking is needed here.
    class pose_server final
    {
    public:
        pose_server(uint16_t port, const app::source_options& initial);

        // blocks: listen + run the event loop
        int run(); 

    private:
        // Pipeline ops (loop thread)
        bool _apply_open(
            const std::string& source, // unsigned int: device index / else: recording path.
            double tag_size_m,
            std::optional<int32_t> exposure_us, 
            std::optional<int32_t> gain
        );
        void _apply_close();
        bool _apply_calibrate();
        void _apply_clear_rest();

        // Pull latched detections and recompute joint states;
        // true if a new frame arrived.
        bool _poll_new_detections();

        // Protocol serializers (loop thread) -> FlatBuffers `Message` bytes.
        std::string _serialize_pose_frame() const;
        std::string _serialize_server_status() const;
        std::string _serialize_ack(bool ok, std::string_view message) const;

    private:
        uint16_t _port;
        app::source_options _initial;

        std::shared_ptr<hw::sensor_frame_provider> _provider;
        std::shared_ptr<pose_frame_observer> _observer;
        pose::pose_estimator _estimator;

        std::vector<pose::tag_detection_t> _detections;
        uint64_t _last_seq{ 0 };
        bool _is_recording{ false };
        size_t _client_count{ 0 }; // connected clients; source is released when it hits 0
    };

} // namespace net

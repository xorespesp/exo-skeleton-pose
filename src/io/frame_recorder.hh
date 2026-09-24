#pragma once
#include "recording_writer.hh"

#include "hw/frameset_observer.hh"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace io
{
    // 관찰한 synced_frameset 을 녹화 파일에 쓴다.
    // 스트림 인덱스 i 의 슬롯은 파일의 스트림 i 가 되고, 슬롯마다 자기 캡처의 타임스탬프로 쓰인다.
    // 관찰자 콜백은 큐에 넣기만 하고 워커 스레드 하나가 모든 스트림을 인코딩한다.
    // 큐는 유한하기 때문에, 인코딩이 밀리면 frameset 이 버려진다. (stats()로 확인)
    class frame_recorder final : public hw::synced_frameset_observer
    {
    public:
        static constexpr std::size_t kDefaultQueueDepth = 8; // 스트림당

        explicit frame_recorder(
            const recording_options_t& options = {},
            std::size_t queue_depth_per_stream = kDefaultQueueDepth);
        ~frame_recorder() override;

        // 파일을 열고 스트림을 인덱스 순으로 등록한다. 다음 frameset 부터 녹화된다. 빈 목록은 거절.
        [[nodiscard]] bool start(
            const std::filesystem::path& path,
            std::span<const camera_stream_info_t> stream_infos
        ) noexcept;

        // 큐에 남은 것을 다 쓰고 파일을 닫는다. 멱등.
        void stop() noexcept;

        bool is_started() const noexcept { return _is_started.load(std::memory_order_relaxed); }
        std::size_t stream_count() const noexcept { return _stream_count; }
        recording_stats_t stats() const noexcept;
        const std::filesystem::path& path() const noexcept { return _writer.path(); }

        void on_synced_frameset_update(const hw::synced_frameset& new_frameset) override;
        void on_sensor_stream_reset() override;
        void on_sensor_frame_geometry_changed(std::size_t stream_idx) override;
        void on_sensor_stream_end(hw::stream_end_reason_t reason) override;

    private:
        struct queued_frame_t
        {
            std::size_t stream_idx{ 0 };
            std::shared_ptr<hw::sensor_frame> frame;
        };

        void _worker(std::stop_token stop);

    private:
        const std::size_t _queue_depth_per_stream;

        recording_writer _writer;
        std::size_t _stream_count{ 0 }; // start() 에서 정해지고 stop() 까지 고정. writer 의 스트림 인덱스와 슬롯 인덱스가 같다
        std::size_t _queue_depth{ 0 };  // _queue_depth_per_stream * 스트림 수

        mutable std::mutex _mtx;
        std::condition_variable_any _queue_cv;
        std::deque<queued_frame_t> _queue;
        uint64_t _frames_dropped{ 0 };

        std::atomic_bool _is_started{ false };
        std::jthread _thread;
        std::mutex _stop_mtx;
    };

} // namespace io

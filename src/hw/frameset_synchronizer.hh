#pragma once
#include "calibration.hh"
#include "frame_format.hh"
#include "roi.hh"
#include "sensor_frame_source.hh"
#include "synced_frameset.hh"
#include "timestamp.hh"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace hw
{
    // 링에서 무엇을 집고, 링이 찼을 때 어떻게 하는지. 소스가 실시간인지 파일인지에 따라 고른다.
    enum class ring_policy_t : uint8_t
    {
        // 소비자는 가장 새로운 캡처를 집고 그보다 오래된 것은 버린다. 링이 차면 가장 오래된 것을 버린다.
        // 소비자가 느려지면 처리율만 내려간다. 라이브 카메라용.
        newest_first,

        // 소비자는 가장 오래된 캡처부터 집는다. 링이 차면 grabber 가 자리를 기다린다. 한 장도 건너뛰지
        // 않으므로 재생용.
        in_order,
    };

    // 여러 스트림의 캡처를 한 순간으로 묶는 방법.
    struct sync_options_t
    {
        // 다른 모든 스트림이 시각을 맞추는 기준 스트림. 묶음은 이 스트림의 캡처마다 하나다.
        std::size_t reference_stream_idx{ 0 };

        // 캡처가 기준에서 이만큼까지 떨어져도 같은 순간으로 친다. 비어 있으면 기준 스트림의 프레임
        // 간격(최근 것들의 중앙값)의 절반이고, 그 거리 안의 최근접 캡처는 하나뿐이다.
        std::optional<std::chrono::nanoseconds> max_pair_skew;

        // 기준 캡처와 같은 순간의 캡처가 아직 안 왔을 때 기다리는 최대 시간. 비어 있으면 기준 간격 하나.
        std::optional<std::chrono::nanoseconds> match_deadline;

        ring_policy_t ring_policy{ ring_policy_t::newest_first };

        // 스트림마다 들고 있는 최근 캡처 수. 소비자가 이만큼의 프레임 시간 안에 돌아오면 같은 순간의
        // 캡처를 놓치지 않는다.
        std::size_t ring_depth_frames{ 4 };
    };

    // synchronizer 가 만들어진 뒤의 카운터.
    struct sync_stats_t
    {
        struct per_stream_t
        {
            uint64_t frames_fetched{ 0 };
            uint64_t frames_dropped{ 0 };                 // 더 새로운 캡처에 밀렸거나, 맞는 기준 캡처가 없었던 것
            std::chrono::nanoseconds last_pair_skew{ 0 }; // 마지막으로 묶였을 때 이 스트림과 기준의 시각 차
        };
        std::vector<per_stream_t> per_stream;

        uint64_t framesets_emitted{ 0 };
        uint64_t framesets_dropped_incomplete{ 0 };   // 슬롯을 다 못 채워 기준 캡처째 버린 것
        uint64_t framesets_dropped_out_of_order{ 0 }; // 직전 묶음보다 뒤로 간 것

        std::optional<std::chrono::nanoseconds> pair_tolerance; // 지금 걸려 있는 허용오차. 도출 전에는 비어 있다
    };

    // 하나 이상의 스트림에서 캡처를 모아 `synced_frameset` 으로 묶는다. 기준 스트림의 캡처마다 하나다.
    //
    // 스트림마다 grabber 스레드가 `fetch` 를 쉬지 않고 돌려 링에 넣는다. 카메라 쪽 버퍼는 소비자 속도와
    // 무관하게 비워지고, 버릴 것은 여기의 링에서 `ring_policy_t` 대로 버린다. 여러 카메라의 변환(demosaic)은
    // 각자의 grabber 에서 나란히 돈다.
    //
    // `fetch_next_synced_frameset()` 은 호출자의 스레드에서 기준 링의 캡처를 집고, 다른 링에서 그 시각에 가장
    // 가까운 캡처를 허용오차 안에서 고른다. 아직 안 왔을 수 있으면 `match_deadline` 만큼 기다린다. 어느
    // 스트림이든 못 채우면 그 묶음은 기준 캡처째 버리고 카운터만 올린다. 나가는 묶음은 언제나 모든 스트림이
    // 채워져 있다.
    //
    // 기준 스트림이 아무것도 내지 않으면 `fetch` 는 비어 있는 값을 돌려주고, 호출자가 다시 부르면 grabber 가
    // 다시 시도한다. 소스가 던지면 grabber 가 그 예외를 들고 있다가 `fetch` 에서 호출자의 스레드로 다시
    // 던진다.
    //
    // 묶음의 시각은 가장 이른 캡처의 것이고, 직전 묶음보다 뒤로 가는 묶음은 나가지 않는다.
    class frameset_synchronizer final
    {
    public:
        // 스트림 하나. 캡처가 오는 대로 하나짜리 묶음으로 나간다.
        frameset_synchronizer(std::shared_ptr<sensor_frame_source> stream, ring_policy_t ring_policy);

        // 여러 스트림을 한 순간으로. `options.reference_stream_idx` 는 `streams` 안을 가리켜야 한다.
        frameset_synchronizer(
            std::vector<std::shared_ptr<sensor_frame_source>> streams,
            const sync_options_t& options
        );

        ~frameset_synchronizer();

        frameset_synchronizer(const frameset_synchronizer&) = delete;
        frameset_synchronizer& operator=(const frameset_synchronizer&) = delete;

        std::size_t stream_count() const noexcept { return _slots.size(); }

        const calibration_t& get_calibration(std::size_t stream_idx) const;
        frame_format_t get_frame_format(std::size_t stream_idx) const;
        stream_descriptor_t get_stream_descriptor(std::size_t stream_idx) const;

        // `stream_idx` 의 소스에 넘긴다. 쓰는 동안 grabber 들이 멈춰 있고 들고 있던 캡처는 전부 버려지므로,
        // 옛 픽셀 프레임의 캡처가 새 픽셀 프레임의 캡처와 같은 묶음에 들어가지 않는다.
        std::optional<roi_t> try_set_roi(std::size_t stream_idx, const roi_t& roi);

        // 기준 링에 캡처가 올 때까지 기다린다. 기준 스트림이 아무것도 내지 않으면 비어 있다: 녹화라면
        // 끝이고, 카메라라면 fetch 가 타임아웃한 것이다.
        [[nodiscard]] std::optional<synced_frameset> fetch_next_synced_frameset();

        // 스트림들의 위치를 옮기는 `move_streams` 를 grabber 가 하나도 fetch 안에 있지 않을 때 실행하고,
        // 링에 들고 있던 것을 전부 버린다. 재개한 grabber 의 첫 read 가 곧 새 위치의 첫 캡처이고, 다음
        // fetch 가 거기서 싱크를 잡는다. 진행 중이던 fetch 하나가 끝날 때까지 기다린다.
        void reposition(const std::function<void()>& move_streams);

        // grabber 들을 멈추고 모든 스트림을 닫는다.
        void close();

        sync_stats_t get_sync_stats() const;

    private:
        struct stream_slot_t
        {
            std::shared_ptr<sensor_frame_source> source;
            std::deque<sensor_frameset> ring; // 오래된 것부터
            uint64_t pushes{ 0 };             // 링에 넣은 횟수. 소비자가 새 캡처를 기다릴 때 본다
            bool exhausted{ false };          // grabber 가 아무것도 못 받았고 소비자가 아직 그것을 보지 않았다
            sync_stats_t::per_stream_t stats;
            std::jthread grabber;
        };

        void _grab_loop(std::size_t stream_idx, std::stop_token stop);

        // 이하 전부 `_mtx` 아래에서 부른다.
        std::optional<std::chrono::nanoseconds> _pair_tolerance() const;
        std::chrono::nanoseconds _match_deadline() const;
        void _observe_reference_interval(timestamp_t reference_timestamp);
        sensor_frameset _take_reference(stream_slot_t& slot);
        std::optional<sensor_frameset> _take_partner(
            stream_slot_t& slot,
            timestamp_t reference_timestamp,
            std::optional<std::chrono::nanoseconds> tolerance
        );
        void _acknowledge_exhaustion(stream_slot_t& slot);

        std::vector<std::unique_ptr<stream_slot_t>> _slots; // grabber 가 자기 슬롯을 참조하므로 주소를 고정한다
        sync_options_t _options;

        mutable std::mutex _mtx;
        std::condition_variable_any _cv;
        bool _closing{ false };

        // `reposition()` 이 도는 동안 참. grabber 는 이것이 내려갈 때까지 fetch 에 들어가지 않는다.
        bool _repositioning{ false };
        std::size_t _grabbers_in_fetch{ 0 }; // 지금 소스의 fetch 안에 있는 grabber 수

        // 허용오차의 출처. 최근 간격들의 중앙값이라 드롭으로 벌어진 간격이 섞여도 실제 주기가 나온다.
        // 중앙값은 reposition 을 넘어 남고, 마지막 시각은 seek 이 끊으므로 reposition 에서 비운다.
        std::optional<timestamp_t> _last_fetched_reference_timestamp;
        std::deque<std::chrono::nanoseconds> _reference_intervals;
        std::optional<std::chrono::nanoseconds> _reference_interval; // 위의 중앙값

        std::optional<timestamp_t> _last_emitted_timestamp;

        // grabber 가 잡은 예외. 소비자의 fetch 가 다시 던진다.
        std::exception_ptr _failure;

        uint64_t _framesets_emitted{ 0 };
        uint64_t _framesets_dropped_incomplete{ 0 };
        uint64_t _framesets_dropped_out_of_order{ 0 };
    };

} // namespace hw

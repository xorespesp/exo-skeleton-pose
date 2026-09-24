#pragma once
#include "joint_measurement.hh"
#include "marker_tracker.hh"

#include "hw/sensor_frameset.hh"
#include "hw/synced_frameset.hh"
#include "hw/timestamp.hh"

#include <BS_thread_pool.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

namespace pose
{
    class synced_tracker_base
    {
    public:
        // 스트림 하나의 검출 통계.
        struct stream_stats_t
        {
            uint64_t frames_processed{ 0 };
            double last_process_ms{ 0.0 };
            double process_ms_ema{ 0.0 };
        };

        // frameset 단위 통계. 처리 시간은 가장 느린 슬롯에 풀의 오버헤드를 더한 것이다.
        struct stats_t
        {
            uint64_t framesets_submitted{ 0 };
            uint64_t framesets_processed{ 0 };
            uint64_t framesets_dropped{ 0 };  // 처리되기 전에 더 새로운 frameset 에 밀린 것
            uint64_t framesets_failed{ 0 };   // 슬롯 task 가 던져서 발행하지 못한 것
            uint64_t measurements_dropped{ 0 }; // 소비되기 전에 더 새로운 묶음에 밀린 것
            double last_process_ms{ 0.0 };
            double process_ms_ema{ 0.0 };
            float process_rate_fps{ 0.0f }; // EMA
        };

        // 스트림 하나: 그 트래커와 그 카메라의 뷰. 뷰는 발행하는 슬롯에 그대로 실린다.
        struct stream_entry_t
        {
            std::shared_ptr<marker_tracker_base> tracker;
            camera_view_t view{ camera_view_t::frontal };
        };

        virtual ~synced_tracker_base() = default;

        synced_tracker_base(const synced_tracker_base&) = delete;
        synced_tracker_base& operator=(const synced_tracker_base&) = delete;

        virtual std::size_t stream_count() const noexcept = 0;

        // 범위 밖 인덱스는 던진다.
        virtual marker_tracker_base& tracker(std::size_t stream_idx) = 0;
        virtual const marker_tracker_base& tracker(std::size_t stream_idx) const = 0;

        // 처리를 기다리는 frameset 을 대체한다.
        // 어느 스레드에서든 부를 수 있고 막히지 않는다. 스트림 수가 다른 frameset 은 버린다.
        virtual void submit(hw::synced_frameset frameset) = 0;

        // 스트림 `stream_idx` 가 마지막으로 처리한 프레임의 어노테이트 사본과 원본. 그 프레임의 id 가
        // `last_frame_id` 와 다를 때만 true 이고 `last_frame_id` 를 갱신한다. 비우지 않으므로 읽는 쪽마다 커서를
        // 따로 든다. `annotate` 가 꺼져 있으면 항상 false.
        virtual bool try_get_annotated(std::size_t stream_idx, cv::Mat& annotated, cv::Mat& source, uint64_t& last_frame_id) const = 0;

        virtual stream_stats_t stream_stats(std::size_t stream_idx) const = 0;
        virtual stats_t stats() const = 0;

    protected:
        synced_tracker_base() = default;
    };

    // ---------------------------------------------------------------------------
    // frameset 단위 검출: 스트림마다 트래커 하나, 한 묶음으로 발행
    // ---------------------------------------------------------------------------
    //
    // 전용 스레드가 frameset 을 꺼내 슬롯마다 검출 task 를 풀에 던지고, 모두 끝나면 슬롯별 측정치를 묶음
    // 하나로 발행한다. 한 묶음의 슬롯은 언제나 같은 frameset 의 것이고, 지연은 가장 느린 슬롯의 것이다.
    //
    // 들어오는 frameset 과 발행한 묶음은 모두 newest-wins 다. 밀리면 모든 스트림이 같은 순간을 함께
    // 건너뛴다. frameset 을 하나씩 처리하므로 한 트래커가 두 프레임을 동시에 받지 않는다.
    //
    // 트래커에서 꺼내는 측정치는 `Measurement` 하나다. 트래커의 래치는 꺼내면 비므로 한 프레임에서 한 종류만
    // 꺼낼 수 있다.
    template <typename Measurement>
    class synced_tracker final : public synced_tracker_base
    {
    public:
        // `stream_entries[i]` == 스트림 i. 풀은 스트림 수만큼의 스레드를 든다.
        // 비었거나 트래커가 null 이면 `std::invalid_argument`.
        synced_tracker(
            std::vector<stream_entry_t> stream_entries,
            bool annotate // 처리한 프레임마다 검출을 그린 BGR 사본을 스트림별로 남길지 여부
        );
        ~synced_tracker() override;

        std::size_t stream_count() const noexcept override { return _stream_entries.size(); }

        marker_tracker_base& tracker(std::size_t stream_idx) override;
        const marker_tracker_base& tracker(std::size_t stream_idx) const override;

        void submit(hw::synced_frameset frameset) override;

        // 가장 최근에 발행된 묶음을 가져간다. 새로 발행된 것이 없으면 false.
        bool try_take_measurements(synced_measurements_t<Measurement>& out);

        bool try_get_annotated(std::size_t stream_idx, cv::Mat& annotated, cv::Mat& source, uint64_t& last_frame_id) const override;

        stream_stats_t stream_stats(std::size_t stream_idx) const override;
        stats_t stats() const override;

    private:
        void _run(std::stop_token stop);

        // 슬롯마다 검출을 풀에 던지고 전부 기다린 뒤 묶음을 발행한다.
        void _process_frameset(const hw::synced_frameset& frameset, uint64_t seq);

        // 슬롯 하나의 검출. 풀의 워커에서 돈다.
        void _process_slot(
            std::size_t stream_idx,
            const hw::sensor_frameset& capture,
            typename synced_measurements_t<Measurement>::slot_t& out_slot
        );

        struct stream_state_t
        {
            stream_stats_t stats;
            cv::Mat annotated;
            cv::Mat source;
            uint64_t annotated_frame_id{ 0 }; // 0: 아직 래치된 프레임이 없다
        };

        std::vector<stream_entry_t> _stream_entries;
        const bool _annotate;

        mutable std::mutex _mtx; // 아래 전부. 슬롯 task 들이 자기 스트림의 상태를 갱신할 때도 잡는다
        std::condition_variable_any _cv;
        std::optional<hw::synced_frameset> _pending;
        std::optional<synced_measurements_t<Measurement>> _published; // 비어 있으면 가져갈 것 없음
        std::vector<stream_state_t> _streams;
        stats_t _stats;
        std::chrono::steady_clock::time_point _last_processed_at{};
        bool _have_last_processed_at{ false };

        BS::thread_pool<> _pool;
        std::jthread _thread; // 마지막에 선언: 위의 것들이 사라지기 전에 join 된다
    };

    extern template class synced_tracker<joint_2d_measurement_t>;
    extern template class synced_tracker<joint_3d_measurement_t>;

} // namespace pose

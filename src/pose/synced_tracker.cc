#include "synced_tracker.hh"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <exception>
#include <future>
#include <stdexcept>
#include <utility>

namespace pose
{
    namespace
    {
        constexpr double kEmaAlpha = 0.1; // 통계에서 최신 표본의 가중치

        double ema(const double current, const double sample)
        {
            return (current <= 0.0) ? sample : (kEmaAlpha * sample + (1.0 - kEmaAlpha) * current);
        }

        // 스레드와 풀이 만들어지기 전에 검사하므로, 던지면 아무것도 돌고 있지 않다.
        std::vector<synced_tracker_base::stream_entry_t> checked(std::vector<synced_tracker_base::stream_entry_t> stream_entries)
        {
            if (stream_entries.empty()) {
                throw std::invalid_argument{ "synced_tracker: no trackers" };
            }
            if (std::ranges::any_of(stream_entries, [](const auto& entry) { return entry.tracker == nullptr; })) {
                throw std::invalid_argument{ "synced_tracker: a tracker is null" };
            }
            return stream_entries;
        }

        // 트래커가 방금 발행한 프레임의 측정치를 그 타입으로 꺼낸다.
        bool take_latest(marker_tracker_base& tracker, std::vector<joint_2d_measurement_t>& out)
        {
            return tracker.try_get_2d_measurements(out);
        }

        bool take_latest(marker_tracker_base& tracker, std::vector<joint_3d_measurement_t>& out)
        {
            return tracker.try_get_3d_measurements(out);
        }
    } // namespace

    template <typename Measurement>
    synced_tracker<Measurement>::synced_tracker(
        std::vector<stream_entry_t> stream_entries,
        const bool annotate)
        : _stream_entries{ checked(std::move(stream_entries)) }
        , _annotate{ annotate }
        , _streams(_stream_entries.size())
        , _pool{ _stream_entries.size() }
        , _thread{ [this](std::stop_token stop) { this->_run(stop); } }
    { }

    template <typename Measurement>
    synced_tracker<Measurement>::~synced_tracker()
    {
        _thread.request_stop(); // 대기 중이면 stop_token 이 깨운다
    }

    template <typename Measurement>
    marker_tracker_base& synced_tracker<Measurement>::tracker(const std::size_t stream_idx)
    {
        return *_stream_entries.at(stream_idx).tracker;
    }

    template <typename Measurement>
    const marker_tracker_base& synced_tracker<Measurement>::tracker(const std::size_t stream_idx) const
    {
        return *_stream_entries.at(stream_idx).tracker;
    }

    template <typename Measurement>
    void synced_tracker<Measurement>::submit(hw::synced_frameset frameset)
    {
        if (frameset.stream_count() != _stream_entries.size())
        {
            spdlog::warn("synced_tracker: a frameset with {} stream(s) was submitted to {} tracker(s); dropped",
                frameset.stream_count(), _stream_entries.size());
            return;
        }
        {
            std::scoped_lock lk{ _mtx };
            ++_stats.framesets_submitted;
            if (_pending.has_value()) { ++_stats.framesets_dropped; }
            _pending = std::move(frameset);
        }
        _cv.notify_one();
    }

    template <typename Measurement>
    bool synced_tracker<Measurement>::try_take_measurements(synced_measurements_t<Measurement>& out)
    {
        std::scoped_lock lk{ _mtx };
        if (!_published.has_value()) { return false; }
        out = std::move(_published.value());
        _published.reset();
        return true;
    }

    template <typename Measurement>
    bool synced_tracker<Measurement>::try_get_annotated(
        const std::size_t stream_idx,
        cv::Mat& annotated,
        cv::Mat& source,
        uint64_t& last_frame_id) const
    {
        std::scoped_lock lk{ _mtx };
        if (stream_idx >= _streams.size()) { return false; }
        const stream_state_t& state = _streams[stream_idx];
        if (state.annotated_frame_id == 0 || state.annotated_frame_id == last_frame_id) { return false; }
        annotated = state.annotated; // 워커는 다음 프레임을 새 Mat 으로 옮겨 넣으므로 얕은 공유가 안전하다
        source = state.source;
        last_frame_id = state.annotated_frame_id;
        return true;
    }

    template <typename Measurement>
    synced_tracker_base::stream_stats_t synced_tracker<Measurement>::stream_stats(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _mtx };
        return stream_idx < _streams.size() ? _streams[stream_idx].stats : stream_stats_t{};
    }

    template <typename Measurement>
    synced_tracker_base::stats_t synced_tracker<Measurement>::stats() const
    {
        std::scoped_lock lk{ _mtx };
        return _stats;
    }

    template <typename Measurement>
    void synced_tracker<Measurement>::_process_slot(
        const std::size_t stream_idx,
        const hw::sensor_frameset& capture,
        typename synced_measurements_t<Measurement>::slot_t& out_slot)
    {
        const std::shared_ptr<hw::sensor_frame>& frame = capture.frame();
        marker_tracker_base& stream_tracker = *_stream_entries[stream_idx].tracker;

        // 캔버스는 기술과 무관하므로 여기서 준비해 넘긴다. 검출이 그 위에 그린다.
        cv::Mat annotated;
        if (_annotate)
        {
            if (frame->format() == hw::frame_format_t::gray8) {
                cv::cvtColor(frame->image(), annotated, cv::COLOR_GRAY2BGR);
            } else {
                annotated = frame->image().clone();
            }
        }

        const auto started = std::chrono::steady_clock::now();
        stream_tracker.process_frame(
            frame->image(),
            frame->format(),
            _annotate ? &annotated : nullptr
        );

        // 트래커의 래치는 newest-wins 라, 다음 frameset 이 덮기 전에 같은 task 에서 꺼낸다.
        out_slot.measured = take_latest(stream_tracker, out_slot.measurements);
        if (!out_slot.measured) { out_slot.measurements.clear(); }
        out_slot.detection_count = stream_tracker.last_detection_count();
        const auto finished = std::chrono::steady_clock::now();
        const double process_ms = std::chrono::duration<double, std::milli>{ finished - started }.count();

        std::scoped_lock lk{ _mtx };
        stream_state_t& state = _streams[stream_idx];
        ++state.stats.frames_processed;
        state.stats.last_process_ms = process_ms;
        state.stats.process_ms_ema = ema(state.stats.process_ms_ema, process_ms);
        if (_annotate)
        {
            state.annotated = std::move(annotated);
            state.source = frame->image();
            state.annotated_frame_id = frame->id();
        }
    }

    template <typename Measurement>
    void synced_tracker<Measurement>::_run(const std::stop_token stop)
    {
        uint64_t seq = 0;
        while (true)
        {
            hw::synced_frameset frameset;
            {
                std::unique_lock lk{ _mtx };
                _cv.wait(lk, stop, [&] { return _pending.has_value(); });
                if (stop.stop_requested()) { return; }
                frameset = std::move(_pending.value());
                _pending.reset();
            }

            this->_process_frameset(frameset, ++seq);
        }
    }

    template <typename Measurement>
    void synced_tracker<Measurement>::_process_frameset(const hw::synced_frameset& frameset, const uint64_t seq)
    {
        const std::size_t stream_count = frameset.stream_count();
        synced_measurements_t<Measurement> bundle;
        bundle.seq = seq;
        bundle.timestamp = frameset.timestamp();
        bundle.slots.resize(stream_count);

        // 슬롯마다 task 하나. 자기 슬롯의 `slot_t` 에만 쓰고, 공유 상태는 `_mtx` 아래에서만 만진다.
        const auto started = std::chrono::steady_clock::now();
        std::vector<std::future<void>> tasks;
        tasks.reserve(stream_count);
        for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx)
        {
            bundle.slots[stream_idx].view = _stream_entries[stream_idx].view;

            const hw::sensor_frameset* capture = frameset.stream_frameset(stream_idx);
            if (!capture) { continue; }

            tasks.push_back(_pool.submit_task([this, stream_idx, capture, slot = &bundle.slots[stream_idx]] {
                this->_process_slot(stream_idx, *capture, *slot);
            }));
        }

        // 전부 끝날 때까지 기다린다. 던진 task 가 있어도 나머지를 끝까지 기다려야 frameset 과 묶음을
        // 안전하게 놓을 수 있다.
        std::exception_ptr failure;
        for (std::future<void>& task : tasks)
        {
            try { task.get(); }
            catch (...) { if (!failure) { failure = std::current_exception(); } }
        }
        const auto finished = std::chrono::steady_clock::now();
        const double process_ms = std::chrono::duration<double, std::milli>{ finished - started }.count();

        if (failure)
        {
            try { std::rethrow_exception(failure); }
            catch (const std::exception& e) {
                spdlog::error("synced_tracker: frameset #{} failed: {}", bundle.seq, e.what());
            }
            catch (...) {
                spdlog::error("synced_tracker: frameset #{} failed", bundle.seq);
            }
            std::scoped_lock lk{ _mtx };
            ++_stats.framesets_failed;
            return;
        }

        std::scoped_lock lk{ _mtx };
        ++_stats.framesets_processed;
        _stats.last_process_ms = process_ms;
        _stats.process_ms_ema = ema(_stats.process_ms_ema, process_ms);
        if (_have_last_processed_at)
        {
            const double dt_sec = std::chrono::duration<double>{ finished - _last_processed_at }.count();
            if (dt_sec > 1e-9)
            {
                const float inst = static_cast<float>(1.0 / dt_sec);
                _stats.process_rate_fps = (_stats.process_rate_fps <= 0.0f)
                    ? inst
                    : static_cast<float>(kEmaAlpha * inst + (1.0 - kEmaAlpha) * _stats.process_rate_fps);
            }
        }
        _last_processed_at = finished;
        _have_last_processed_at = true;

        if (_published.has_value()) { ++_stats.measurements_dropped; }
        _published = std::move(bundle);
    }

    template class synced_tracker<joint_2d_measurement_t>;
    template class synced_tracker<joint_3d_measurement_t>;

} // namespace pose

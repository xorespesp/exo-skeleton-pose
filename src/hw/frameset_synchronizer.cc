#include "frameset_synchronizer.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace hw
{
    namespace
    {
        // 허용오차를 도출하는 최근 기준 간격의 수. 중앙값이라 이 중 절반 미만이 드롭으로 벌어져 있어도
        // 실제 주기가 나온다.
        constexpr std::size_t kIntervalWindow = 16;

        // 기준 캡처를 기다리는 한도. 백엔드의 grab 타임아웃과 같은 자릿수라, 카메라가 침묵하면 grabber 가
        // 그것을 알린 뒤에 소비자가 깬다.
        constexpr auto kReferenceWaitTimeout = std::chrono::milliseconds{ 1000 };

        // 기준 간격을 아직 모를 때, 상대 캡처를 기다리는 한도.
        constexpr auto kDefaultMatchDeadline = std::chrono::milliseconds{ 50 };

        std::chrono::nanoseconds distance(const timestamp_t a, const timestamp_t b)
        {
            return (a >= b) ? (a - b) : (b - a);
        }
    }

    frameset_synchronizer::frameset_synchronizer(
        std::shared_ptr<sensor_frame_source> stream,
        const ring_policy_t ring_policy)
        : frameset_synchronizer{ [&] {
              std::vector<std::shared_ptr<sensor_frame_source>> streams;
              streams.push_back(std::move(stream));
              return streams;
          }(), sync_options_t{ .ring_policy = ring_policy } }
    { }

    frameset_synchronizer::frameset_synchronizer(
        std::vector<std::shared_ptr<sensor_frame_source>> streams,
        const sync_options_t& options)
        : _options{ options }
    {
        if (streams.empty()) {
            throw std::invalid_argument{ "frameset_synchronizer: no streams" };
        }
        if (_options.reference_stream_idx >= streams.size()) {
            throw std::invalid_argument{ "frameset_synchronizer: reference stream is not among the streams" };
        }
        if (_options.ring_depth_frames == 0) {
            throw std::invalid_argument{ "frameset_synchronizer: ring depth must be at least one" };
        }
        for (auto& stream : streams)
        {
            if (!stream) { throw std::invalid_argument{ "frameset_synchronizer: null stream" }; }
            auto slot = std::make_unique<stream_slot_t>();
            slot->source = std::move(stream);
            _slots.push_back(std::move(slot));
        }

        // 슬롯이 전부 자리를 잡은 뒤에 grabber 를 띄운다. 각자 자기 슬롯만 만지고, 슬롯 벡터는 이후 바뀌지
        // 않는다.
        for (std::size_t stream_idx = 0; stream_idx < _slots.size(); ++stream_idx)
        {
            _slots[stream_idx]->grabber = std::jthread{
                [this, stream_idx](std::stop_token stop) { this->_grab_loop(stream_idx, stop); } };
        }
    }

    frameset_synchronizer::~frameset_synchronizer()
    {
        this->close();
    }

    void frameset_synchronizer::close()
    {
        {
            std::scoped_lock lk{ _mtx };
            if (_closing) { return; }
            _closing = true;
        }
        for (auto& slot : _slots) { slot->grabber.request_stop(); }
        _cv.notify_all();
        for (auto& slot : _slots)
        {
            if (slot->grabber.joinable()) { slot->grabber.join(); } // fetch 안에 있으면 그 타임아웃까지
        }
        for (auto& slot : _slots)
        {
            if (slot->source) { slot->source->close(); }
        }
    }

    const calibration_t& frameset_synchronizer::get_calibration(const std::size_t stream_idx) const
    {
        return _slots.at(stream_idx)->source->get_calibration();
    }

    frame_format_t frameset_synchronizer::get_frame_format(const std::size_t stream_idx) const
    {
        return _slots.at(stream_idx)->source->get_frame_format();
    }

    stream_descriptor_t frameset_synchronizer::get_stream_descriptor(const std::size_t stream_idx) const
    {
        return _slots.at(stream_idx)->source->get_stream_descriptor();
    }

    std::optional<roi_t> frameset_synchronizer::try_set_roi(const std::size_t stream_idx, const roi_t& roi)
    {
        // 소스 자신의 락이 grabber 의 fetch 와 이 쓰기를 직렬화한다. 쓰기가 끝난 뒤의 flush 가, 그 사이
        // 링에 들어온 옛 기하의 캡처와 fetch 도중인 것을 함께 걸러 낸다.
        const std::optional<roi_t> granted = _slots.at(stream_idx)->source->try_set_roi(roi);
        this->flush();
        return granted;
    }

    void frameset_synchronizer::flush()
    {
        {
            std::scoped_lock lk{ _mtx };
            ++_generation;
            for (auto& slot : _slots)
            {
                slot->ring.clear();
                slot->exhausted = false;
            }
            _last_fetched_reference_timestamp.reset();
            _last_emitted_timestamp.reset();
        }
        _cv.notify_all();
    }

    sync_stats_t frameset_synchronizer::get_sync_stats() const
    {
        std::scoped_lock lk{ _mtx };
        sync_stats_t stats;
        stats.per_stream.reserve(_slots.size());
        for (const auto& slot : _slots) { stats.per_stream.push_back(slot->stats); }
        stats.framesets_emitted = _framesets_emitted;
        stats.framesets_dropped_incomplete = _framesets_dropped_incomplete;
        stats.framesets_dropped_out_of_order = _framesets_dropped_out_of_order;
        stats.pair_tolerance = this->_pair_tolerance();
        return stats;
    }

    // ---- grabber 스레드 -------------------------------------------------------------------------

    void frameset_synchronizer::_grab_loop(const std::size_t stream_idx, const std::stop_token stop)
    {
        stream_slot_t& slot = *_slots[stream_idx];
        const bool is_reference = stream_idx == _options.reference_stream_idx;
        const bool waits_for_room = _options.ring_policy == ring_policy_t::in_order;

        while (!stop.stop_requested())
        {
            uint64_t generation;
            {
                std::unique_lock lk{ _mtx };
                // 아무것도 못 받은 뒤에는 소비자가 그것을 본 다음에야 다시 시도한다. 카메라는 소비자의
                // 재시도 주기로 다시 묻고, 파일은 seek 이 올 때까지 EOF 에 머문다.
                _cv.wait(lk, stop, [&] {
                    return _closing || (!slot.exhausted && (!waits_for_room || slot.ring.size() < _options.ring_depth_frames));
                });
                if (stop.stop_requested() || _closing) { return; }
                generation = _generation;
            }

            std::optional<sensor_frameset> capture;
            try
            {
                capture = slot.source->fetch_next_sensor_frameset(); // 락 밖에서 블로킹
            }
            catch (...)
            {
                // 소스가 이어갈 수 없는 문제. 소비자의 fetch 가 다시 던져 스트림이 failed 로 끝난다.
                std::scoped_lock lk{ _mtx };
                _failure = std::current_exception();
                slot.exhausted = true;
                _cv.notify_all();
                return;
            }

            std::scoped_lock lk{ _mtx };
            if (!capture.has_value())
            {
                slot.exhausted = true;
                _cv.notify_all();
                continue;
            }
            if (generation != _generation) { continue; } // flush 를 가로질러 당긴 옛 위치의 캡처

            ++slot.stats.frames_fetched;
            if (is_reference) { this->_observe_reference_interval(capture->timestamp()); }

            if (slot.ring.size() >= _options.ring_depth_frames)
            {
                // newest_first 만 여기 온다. in_order 는 위에서 자리를 기다렸다.
                slot.ring.pop_front();
                ++slot.stats.frames_dropped;
            }
            slot.ring.push_back(std::move(capture.value()));
            ++slot.pushes;
            _cv.notify_all();
        }
    }

    // ---- 소비자 (호출자의 스레드) ----------------------------------------------------------------

    std::optional<std::chrono::nanoseconds> frameset_synchronizer::_pair_tolerance() const
    {
        if (_options.max_pair_skew.has_value()) { return _options.max_pair_skew; }
        if (_reference_interval.has_value()) { return *_reference_interval / 2; }
        return std::nullopt;
    }

    std::chrono::nanoseconds frameset_synchronizer::_match_deadline() const
    {
        if (_options.match_deadline.has_value()) { return *_options.match_deadline; }
        if (_reference_interval.has_value()) { return *_reference_interval; }
        return kDefaultMatchDeadline;
    }

    void frameset_synchronizer::_observe_reference_interval(const timestamp_t reference_timestamp)
    {
        if (_last_fetched_reference_timestamp.has_value() && reference_timestamp > *_last_fetched_reference_timestamp)
        {
            _reference_intervals.push_back(reference_timestamp - *_last_fetched_reference_timestamp);
            if (_reference_intervals.size() > kIntervalWindow) { _reference_intervals.pop_front(); }

            std::vector<std::chrono::nanoseconds> sorted(_reference_intervals.begin(), _reference_intervals.end());
            std::sort(sorted.begin(), sorted.end());
            _reference_interval = sorted[sorted.size() / 2];
        }
        _last_fetched_reference_timestamp = reference_timestamp;
    }

    void frameset_synchronizer::_acknowledge_exhaustion(stream_slot_t& slot)
    {
        if (!slot.exhausted) { return; }
        slot.exhausted = false;
        _cv.notify_all();
    }

    sensor_frameset frameset_synchronizer::_take_reference(stream_slot_t& slot)
    {
        if (_options.ring_policy == ring_policy_t::in_order)
        {
            sensor_frameset capture = std::move(slot.ring.front());
            slot.ring.pop_front();
            return capture;
        }

        // newest_first: 가장 새로운 것을 집고, 소비자가 따라잡지 못한 나머지는 버린다.
        sensor_frameset capture = std::move(slot.ring.back());
        slot.stats.frames_dropped += slot.ring.size() - 1;
        slot.ring.clear();
        return capture;
    }

    std::optional<sensor_frameset> frameset_synchronizer::_take_partner(
        stream_slot_t& slot,
        const timestamp_t reference_timestamp,
        const std::optional<std::chrono::nanoseconds> tolerance)
    {
        // 기준보다 허용오차 이상 앞선 캡처는 이후의 기준에도 맞지 않는다. 이후의 기준은 더 늦다.
        if (tolerance.has_value())
        {
            while (!slot.ring.empty() && slot.ring.front().timestamp() < reference_timestamp - *tolerance)
            {
                slot.ring.pop_front();
                ++slot.stats.frames_dropped;
            }
        }
        if (slot.ring.empty()) { return std::nullopt; }

        // 허용오차가 기준 간격의 절반이면 그 안에 있는 캡처가 곧 최근접이다. 다음 캡처는 간격에서 그만큼을
        // 뺀 거리에 온다.
        std::size_t nearest_idx = 0;
        for (std::size_t idx = 1; idx < slot.ring.size(); ++idx)
        {
            if (distance(slot.ring[idx].timestamp(), reference_timestamp)
                < distance(slot.ring[nearest_idx].timestamp(), reference_timestamp))
            {
                nearest_idx = idx;
            }
        }
        if (tolerance.has_value() && distance(slot.ring[nearest_idx].timestamp(), reference_timestamp) > *tolerance)
        {
            return std::nullopt;
        }

        sensor_frameset chosen = std::move(slot.ring[nearest_idx]);
        slot.stats.frames_dropped += nearest_idx; // 고른 것보다 오래된 것은 어느 기준에도 못 붙는다
        slot.ring.erase(slot.ring.begin(), slot.ring.begin() + static_cast<std::ptrdiff_t>(nearest_idx) + 1);
        slot.stats.last_pair_skew = distance(chosen.timestamp(), reference_timestamp);
        return chosen;
    }

    std::optional<synced_frameset> frameset_synchronizer::fetch_next_synced_frameset()
    {
        std::unique_lock lk{ _mtx };
        stream_slot_t& reference_slot = *_slots[_options.reference_stream_idx];

        while (true)
        {
            _cv.wait_for(lk, kReferenceWaitTimeout, [&] {
                return _closing || _failure || !reference_slot.ring.empty() || reference_slot.exhausted;
            });
            if (_closing) { return std::nullopt; }
            if (_failure) { std::rethrow_exception(_failure); }
            if (reference_slot.ring.empty())
            {
                // 침묵이든 EOF 든 위로 올린다. grabber 는 호출자가 다시 물을 때 다시 시도한다.
                this->_acknowledge_exhaustion(reference_slot);
                return std::nullopt;
            }

            std::vector<std::optional<sensor_frameset>> slots(_slots.size());
            slots[_options.reference_stream_idx] = this->_take_reference(reference_slot);
            const timestamp_t reference_timestamp = slots[_options.reference_stream_idx]->timestamp();
            const std::optional<std::chrono::nanoseconds> tolerance = this->_pair_tolerance();

            bool complete = true;
            for (std::size_t stream_idx = 0; stream_idx < _slots.size() && complete; ++stream_idx)
            {
                if (stream_idx == _options.reference_stream_idx) { continue; }
                stream_slot_t& slot = *_slots[stream_idx];

                std::optional<sensor_frameset> partner = this->_take_partner(slot, reference_timestamp, tolerance);

                // 아직 안 왔을 수 있다. 이 스트림이 기준에 뒤처져 있으면 다음 캡처가 같은 순간의 것일 수
                // 있으므로, 새 캡처가 들어오거나 스트림이 침묵을 알릴 때까지 한도 안에서 기다린다.
                const auto deadline = std::chrono::steady_clock::now() + this->_match_deadline();
                while (!partner.has_value() && !slot.exhausted && !_closing && !_failure)
                {
                    const uint64_t seen = slot.pushes;
                    if (!_cv.wait_until(lk, deadline, [&] { return slot.pushes != seen || slot.exhausted || _closing || _failure; })) {
                        break;
                    }
                    partner = this->_take_partner(slot, reference_timestamp, tolerance);
                }
                this->_acknowledge_exhaustion(slot);
                if (_closing) { return std::nullopt; }
                if (_failure) { std::rethrow_exception(_failure); }

                if (!partner.has_value()) { complete = false; break; }
                slots[stream_idx] = std::move(partner);
            }

            // 링에서 꺼냈으니 자리를 기다리던 grabber 가 있으면 깨운다.
            _cv.notify_all();

            if (!complete)
            {
                ++_framesets_dropped_incomplete;
                continue;
            }

            synced_frameset group{ std::move(slots) };

            // 각 스트림이 단조이고 묶음의 시각이 가장 이른 캡처의 것이므로 구조상 성립하는 조건이다.
            // 전제를 깨는 소스는 여기서 잡힌다.
            if (_last_emitted_timestamp.has_value() && group.timestamp() <= *_last_emitted_timestamp)
            {
                ++_framesets_dropped_out_of_order;
                spdlog::warn("synchronizer: dropped a frameset running behind the one before it");
                continue;
            }
            _last_emitted_timestamp = group.timestamp();
            ++_framesets_emitted;
            return group;
        }
    }

} // namespace hw

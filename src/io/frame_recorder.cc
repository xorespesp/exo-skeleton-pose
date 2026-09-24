#include "frame_recorder.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <format>
#include <stdexcept>
#include <utility>
#include <vector>

namespace io
{
    frame_recorder::frame_recorder(
        const recording_options_t& options,
        const std::size_t queue_depth_per_stream)
        : _queue_depth_per_stream{ std::max<std::size_t>(1, queue_depth_per_stream) }
        , _writer{ options }
    { }

    frame_recorder::~frame_recorder()
    {
        this->stop();
    }

    bool frame_recorder::start(
        const std::filesystem::path& path,
        const std::span<const camera_stream_info_t> stream_infos) noexcept try
    {
        if (_is_started.load(std::memory_order_relaxed)) {
            throw std::runtime_error{ "frame_recorder: already started" };
        }
        if (stream_infos.empty()) {
            throw std::invalid_argument{ "frame_recorder: no camera stream to record" };
        }

        if (!_writer.open(path)) {
            throw std::runtime_error{ "frame_recorder: failed to open the recording" };
        }

        // 슬롯 순서로 등록하므로 writer 가 매기는 스트림 인덱스가 슬롯 인덱스와 같다.
        for (std::size_t stream_idx = 0; stream_idx < stream_infos.size(); ++stream_idx)
        {
            const std::optional<std::size_t> registered_idx = _writer.add_camera_stream(stream_infos[stream_idx]);
            if (registered_idx != stream_idx) {
                _writer.close();
                throw std::runtime_error{ std::format("frame_recorder: failed to register camera stream {}", stream_idx) };
            }
        }
        _stream_count = stream_infos.size();
        _queue_depth = _queue_depth_per_stream * _stream_count;

        {
            std::scoped_lock lk{ _mtx };
            _queue.clear();
            _frames_dropped = 0;
        }

        _is_started.store(true, std::memory_order_relaxed);
        _thread = std::jthread{ [this](std::stop_token stop) { this->_worker(std::move(stop)); } };

        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("frame_recorder::start failed: {}", e.what());
        return false;
    }

    void frame_recorder::stop() noexcept
    {
        std::scoped_lock stop_lk{ _stop_mtx };

        // Stop accepting frames first, so the worker sees a queue that can only shrink.
        if (!_is_started.exchange(false, std::memory_order_relaxed)) { return; }

        _thread.request_stop(); // the worker drains what is queued, then exits
        if (_thread.joinable()) { _thread.join(); }

        _writer.close();
    }

    void frame_recorder::_worker(std::stop_token stop)
    {
        for (;;) {
            queued_frame_t queued;
            {
                std::unique_lock lk{ _mtx };
                _queue_cv.wait(lk, stop, [this] { return !_queue.empty(); });

                // 여기서 비어 있으면 정지 요청이 왔고 큐를 다 비운 것이다.
                if (_queue.empty()) { return; }

                queued = std::move(_queue.front());
                _queue.pop_front();
            }

            // 인코딩은 락 밖에서. 안 그러면 관찰자 콜백이 push 에서 그 뒤에 막힌다.
            [[maybe_unused]] const bool succeeded = _writer.write_frame(
                queued.stream_idx,
                queued.frame->image(),
                queued.frame->timestamp() // 슬롯마다 자기 캡처의 시각
            );
        }
    }

    void frame_recorder::on_synced_frameset_update(const hw::synced_frameset& new_frameset)
    {
        if (!_is_started.load(std::memory_order_relaxed)) { return; }

        // 등록한 스트림의 슬롯을 모은다. 파일 안의 순간은 모든 스트림이 갖춰져 있어야 하므로, 슬롯 하나라도
        // 없거나 비어 있으면 그 순간은 쓰지 않는다. provider 는 슬롯이 빈 frameset 을 내보내지 않는다.
        std::vector<queued_frame_t> frames;
        frames.reserve(_stream_count);
        for (std::size_t stream_idx = 0; stream_idx < _stream_count; ++stream_idx)
        {
            const hw::sensor_frameset* capture = new_frameset.stream_frameset(stream_idx);
            if (!capture || capture->frame()->image().empty()) { return; }
            frames.push_back(queued_frame_t{ stream_idx, capture->frame() });
        }

        {
            std::scoped_lock lk{ _mtx };
            if (_queue.size() + frames.size() > _queue_depth) {
                // 인코딩이 밀렸다. 순간을 통째로 버려 메모리를 묶어 두고, 잃은 수를 기록한다.
                _frames_dropped += frames.size();
                return;
            }
            for (queued_frame_t& queued : frames) { _queue.push_back(std::move(queued)); }
        }
        _queue_cv.notify_one();
    }

    void frame_recorder::on_sensor_stream_reset()
    {
        // seek 이나 새 소스는 녹화를 끝내지 않는다. 언제 멈출지는 소유자가 정한다.
    }

    void frame_recorder::on_sensor_frame_geometry_changed(const std::size_t stream_idx)
    {
        // 스트림은 프레임 크기까지 든 캘리브레이션 하나로 선언되어 다른 크기의 프레임은 들어갈 수 없다.
        // 어느 스트림이든 하나가 바뀌면 파일 전체를 닫아 지금까지 담긴 것을 읽을 수 있게 남긴다.
        if (!_is_started.load(std::memory_order_relaxed)) { return; }

        spdlog::warn("recorder: the frame geometry of stream {} changed; finalizing '{}'"
            , stream_idx, _writer.path().string());
        this->stop();
    }

    void frame_recorder::on_sensor_stream_end(const hw::stream_end_reason_t)
    {
        this->stop();
    }

    recording_stats_t frame_recorder::stats() const noexcept
    {
        recording_stats_t stats = _writer.stats(); // safe to read while the worker writes

        std::scoped_lock lk{ _mtx };
        stats.frames_dropped = _frames_dropped;
        return stats;
    }

} // namespace io

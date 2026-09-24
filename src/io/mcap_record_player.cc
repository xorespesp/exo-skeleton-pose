#include "mcap_record_player.hh"

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <utility>

namespace io
{
    // 녹화 파일의 스트림 하나.
    class mcap_record_player::recording_stream_impl final
        : public hw::sensor_frame_source
    {
    public:
        recording_stream_impl(
            std::weak_ptr<mcap_record_player> player, 
            const recorded_camera_stream_t& stream)
            : _player{ std::move(player) }
            , _stream_idx{ stream.stream_idx }
            , _descriptor{ .sensor_backend = stream.stream_info.sensor_backend, .device_serial = stream.stream_info.device_serial }
            , _calibration{ stream.stream_info.calibration }
            , _frame_format{ stream.stream_info.color_format }
            , _codec{ stream.codec }
        { }

        bool is_valid() const override
        {
            const std::shared_ptr<mcap_record_player> player = _player.lock();
            return player && player->is_opened();
        }

        // NOTE: no-op. 파일은 player에서 닫는다.
        void close() override {}

        const hw::calibration_t& get_calibration() const override { return _calibration; }
        hw::frame_format_t get_frame_format() const override { return _frame_format; }
        hw::stream_descriptor_t get_stream_descriptor() const override { return _descriptor; }

        // 녹화는 전체 프레임을 담고 있으므로 소프트웨어 크롭이다.
        std::optional<hw::roi_t> try_set_roi(const hw::roi_t& roi) override
        {
            const hw::roi_t clipped = hw::clamp_roi(roi,
                _calibration.frame_resolution.x(),
                _calibration.frame_resolution.y()
            );
            if (clipped.is_empty()) { return std::nullopt; }

            const std::shared_ptr<mcap_record_player> player = _player.lock();
            if (!player) { return std::nullopt; }

            std::scoped_lock lk{ player->_mtx };
            _roi = clipped;
            return _roi;
        }

        [[nodiscard]] std::optional<hw::sensor_frameset> fetch_next_sensor_frameset() override
        {
            const std::shared_ptr<mcap_record_player> player = _player.lock();
            if (!player) { return std::nullopt; }

            while (true)
            {
                // 파일에서 꺼내는 것만 락 아래에서 한다. 디코드는 스트림마다 자기 grabber 에서 나란히 돈다.
                std::optional<recording_reader::encoded_frame_t> encoded;
                std::optional<hw::roi_t> roi;
                {
                    std::scoped_lock lk{ player->_mtx };
                    if (!player->_opened) { return std::nullopt; }
                    encoded = player->_reader.fetch_next_encoded_frame(_stream_idx);
                    roi = _roi;
                }
                if (!encoded.has_value()) { return std::nullopt; } // EOF

                cv::Mat image = decode_frame(_codec, encoded->payload, _frame_format);
                if (image.empty()) { continue; } // already logged; skip the bad frame rather than end playback

                if (roi.has_value()) {
                    image = image(cv::Rect{ roi->x, roi->y, roi->width, roi->height });
                }

                return hw::sensor_frameset{ std::make_shared<hw::sensor_frame>(
                    std::move(image),
                    _frame_format,
                    encoded->timestamp
                ) };
            }
        }

    private:
        std::weak_ptr<mcap_record_player> _player;
        const std::size_t _stream_idx;
        const hw::stream_descriptor_t _descriptor;
        const hw::calibration_t _calibration; // 전체 프레임 기준
        const hw::frame_format_t _frame_format;
        const image_codec_t _codec;
        std::optional<hw::roi_t> _roi; // nullopt: 전체 프레임
    };

    std::shared_ptr<mcap_record_player> mcap_record_player::create()
    {
        return std::shared_ptr<mcap_record_player>{ new mcap_record_player{} };
    }

    mcap_record_player::~mcap_record_player()
    {
        this->close();
    }

    bool mcap_record_player::open(const std::filesystem::path& recording_file) noexcept try
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) { throw std::runtime_error{ "mcap_record_player: already opened" }; }

        if (!_reader.open(recording_file)) {
            throw std::runtime_error{ "mcap_record_player: failed to open recording" };
        }

        _recording_streams.clear();
        for (const recorded_camera_stream_t& stream : _reader.camera_streams())
        {
            _recording_streams.push_back(std::make_shared<recording_stream_impl>(this->weak_from_this(), stream));
        }

        _first_timestamp = _reader.first_timestamp();
        _last_timestamp = _reader.last_timestamp();
        _opened = true;

        spdlog::info("recording playback ready ({} stream(s), length: {} ms)"
            , _recording_streams.size()
            , std::chrono::duration_cast<std::chrono::milliseconds>(_last_timestamp - _first_timestamp).count()
        );
        for (const recorded_camera_stream_t& stream : _reader.camera_streams())
        {
            spdlog::info("recording stream {}: {}, {} '{}'"
                , stream.stream_idx
                , hw::frame_format_to_str(stream.stream_info.color_format)
                , hw::sensor_backend_to_str(stream.stream_info.sensor_backend)
                , stream.stream_info.device_serial
            );
        }
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("mcap_record_player::open failed: {}", e.what());
        return false;
    }

    bool mcap_record_player::is_opened() const
    {
        std::scoped_lock lk{ _mtx };
        return _opened;
    }

    void mcap_record_player::close()
    {
        std::scoped_lock lk{ _mtx };
        if (_opened) {
            _reader.close();
            _recording_streams.clear();
            _opened = false;
        }
    }

    void mcap_record_player::seek_begin()
    {
        this->seek_timestamp(_first_timestamp);
    }

    void mcap_record_player::seek_end()
    {
        this->seek_timestamp(_last_timestamp);
    }

    void mcap_record_player::seek_timestamp(const hw::timestamp_t timestamp)
    {
        std::scoped_lock lk{ _mtx };
        if (!_opened) { return; }
        for (const recorded_camera_stream_t& stream : _reader.camera_streams()) {
            _reader.seek_timestamp(stream.stream_idx, timestamp);
        }
    }

} // namespace io

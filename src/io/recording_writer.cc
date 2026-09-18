#include "recording_writer.hh"

#include <mcap/writer.hpp>
#include <spdlog/spdlog.h>

#include <charconv>
#include <format>
#include <stdexcept>

namespace io
{
    namespace
    {
        // Current Recording Format Version (YYMMDDRR)
        constexpr std::string_view kFormatVersion{ "26091800" };

        constexpr std::string_view kStreamNamePrefix{ "color" };

        // FlatBuffers is the message encoding for every channel we write.
        constexpr std::string_view kMessageEncoding{ "flatbuffer" };

        // Chunks are the unit of both compression and the seek index: small enough that a
        // seek decompresses little, large enough that raw frames still compress well.
        constexpr uint64_t kChunkSize = 2 * 1024 * 1024;

        mcap::ByteArray to_byte_array(std::span<const uint8_t> bytes)
        {
            const auto* first = reinterpret_cast<const std::byte*>(bytes.data());
            return mcap::ByteArray{ first, first + bytes.size() };
        }

        // ISO 8601 in UTC, e.g. "2026-08-02T14:22:07Z".
        std::string now_iso8601()
        {
            return std::format("{:%FT%TZ}",
                std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now()));
        }
    } // namespace

    std::string stream_name_of(const std::size_t stream_idx)
    {
        return std::format("{}{}", kStreamNamePrefix, stream_idx);
    }

    std::optional<std::size_t> stream_idx_of(std::string_view stream_name) noexcept
    {
        if (!stream_name.starts_with(kStreamNamePrefix)) { return std::nullopt; }
        stream_name.remove_prefix(kStreamNamePrefix.size());

        // Only the digits stream_name_of() prints: no sign, no leading zero, nothing trailing.
        if (stream_name.empty() || (stream_name.size() > 1 && stream_name.front() == '0')) { return std::nullopt; }

        std::size_t stream_idx = 0;
        const auto [end, ec] = std::from_chars(stream_name.data(), stream_name.data() + stream_name.size(), stream_idx);
        if (ec != std::errc{} || end != stream_name.data() + stream_name.size()) { return std::nullopt; }
        return stream_idx;
    }

    recording_writer::recording_writer(const recording_options_t& options)
        : _options{ options }
    { }

    recording_writer::~recording_writer()
    {
        this->close();
    }

    bool recording_writer::open(const std::filesystem::path& path) noexcept try
    {
        if (_writer) { throw std::runtime_error{ "recording_writer: already opened" }; }

        const image_codec_desc_t* codec = find_image_codec(_options.codec);
        if (!codec) { throw std::invalid_argument{ "recording_writer: unknown image codec" }; }

        mcap::McapWriterOptions options{ "" }; // no well-known profile; the schemas describe the messages
        options.library = "exo-skeleton-pose";
        options.chunkSize = kChunkSize;
        options.compression = codec->chunk_compress ? mcap::Compression::Zstd : mcap::Compression::None;
        options.compressionLevel = mcap::CompressionLevel::Fastest; // recording runs live; do not stall the encoder

        auto writer = std::make_unique<mcap::McapWriter>();
        if (const mcap::Status status = writer->open(path.string(), options); !status.ok()) {
            throw std::runtime_error{ std::format(
                "recording_writer: failed to open '{}': {}", path.string(), status.message) };
        }

        // The schemas travel with the recording, so anything that speaks MCAP can decode it.
        mcap::Schema image_schema{
            codec->schema_name, kMessageEncoding, to_byte_array(image_schema_bytes(_options.codec))
        };
        writer->addSchema(image_schema);

        mcap::Schema calibration_schema{
            kCalibrationSchemaName, kMessageEncoding, to_byte_array(calibration_schema_bytes())
        };
        writer->addSchema(calibration_schema);

        writer->write(mcap::Metadata{
            .name = "exo/recording",
            .metadata = {
                { "format_version", std::string{ kFormatVersion } },
                { "create_timestamp", now_iso8601() },
            },
        });

        _writer = std::move(writer);
        _path = path;
        _image_schema_id = image_schema.id;
        _calibration_schema_id = calibration_schema.id;
        _streams.clear();
        _frames_written.store(0, std::memory_order_relaxed);
        _payload_bytes.store(0, std::memory_order_relaxed);
        _has_frames.store(false, std::memory_order_relaxed);

        spdlog::info("recording opened: {} (codec: {})", path.string(), codec->id);
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::open failed: {}", e.what());
        return false;
    }

    std::optional<std::size_t> recording_writer::add_camera_stream(const camera_stream_info_t& info) noexcept try
    {
        if (!_writer) { throw std::runtime_error{ "recording_writer: not opened" }; }

        const image_codec_desc_t* codec = find_image_codec(_options.codec);
        if (!codec) { throw std::invalid_argument{ "recording_writer: unknown image codec" }; }

        // Named by position, so the name states the stream's index and is unique by construction.
        const std::string stream_name = stream_name_of(_streams.size());

        // Everything about this stream sits on its image channel:
        // how to decode it, what layout it carries, and which camera produced it.
        mcap::KeyValueMap image_metadata{
            { "codec", std::string{ codec->id } },
            { "color_format", std::string{ hw::frame_format_to_str(info.color_format) } },
            { "sensor_backend", std::string{ hw::sensor_backend_to_str(info.sensor_backend) } },
            { "device_serial", info.device_serial },
            { "exposure_us", encode_camera_setting(info.exposure_us) },
            { "gain", encode_camera_setting(info.gain) },
        };

        // Written only for the codec it steers, so no reader has to wonder what it meant.
        if (_options.codec == image_codec_t::jpeg) {
            image_metadata.emplace("jpeg_quality", std::to_string(_options.encode.jpeg_quality));
        }

        mcap::Channel image_channel{
            /*topic*/std::format("/camera/{}/image", stream_name),
            /*messageEncoding*/kMessageEncoding,
            /*schemaId*/_image_schema_id,
            /*metadata*/std::move(image_metadata)
        };
        _writer->addChannel(image_channel);

        // NOTE: The calibration travels in the message itself, so the channel carries no metadata.
        mcap::Channel calibration_channel{
            /*topic*/std::format("/camera/{}/calibration", stream_name),
            /*messageEncoding*/kMessageEncoding,
            /*schemaId*/_calibration_schema_id
        };
        _writer->addChannel(calibration_channel);

        _streams.push_back(camera_stream_t{
            .stream_name = stream_name,
            .color_format = info.color_format,
            .image_channel_id = image_channel.id,
            .calibration_channel_id = calibration_channel.id,
            .calibration = info.calibration,
        });

        spdlog::info("recording: camera stream '{}' registered ({}x{}, {}, {} '{}')"
            , stream_name
            , info.calibration.frame_resolution.x()
            , info.calibration.frame_resolution.y()
            , hw::frame_format_to_str(info.color_format)
            , hw::sensor_backend_to_str(info.sensor_backend)
            , info.device_serial
        );

        return _streams.size() - 1;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::add_camera_stream failed: {}", e.what());
        return std::nullopt;
    }

    bool recording_writer::write_frame(
        const std::size_t stream_idx,
        const cv::Mat& image,
        const hw::timestamp_t timestamp) noexcept try
    {
        if (!_writer) { throw std::runtime_error{ "recording_writer: not opened" }; }
        if (stream_idx >= _streams.size()) { throw std::invalid_argument{ "recording_writer: unknown camera stream" }; }

        camera_stream_t& s = _streams[stream_idx];

        // The stream declares one calibration for the whole file, frame size included, so a frame
        // of another size cannot join it: the file would say one thing and hold another.
        if (const Eigen::Vector2i& declared = s.calibration.frame_resolution;
            image.cols != declared.x() || image.rows != declared.y())
        {
            spdlog::error("recording_writer: '{}' declares {}x{} frames; {}x{} turned away"
                , s.stream_name, declared.x(), declared.y(), image.cols, image.rows);
            return false;
        }

        // Encoded on the first frame so it carries that frame's time,
        // which keeps the recording's time range exactly the range of its frames.
        if (!s.calibration_written) {
            const std::vector<std::byte> calibration_payload = encode_calibration(
                s.calibration,
                /*timestamp*/timestamp,
                // The stream's name is also its coordinate frame, 
                // so the calibration and the images below carry the same one and a viewer pairs them up.
                /*coord_frame_id*/s.stream_name
            );
            this->_write_mcap_message(s.calibration_channel_id, 0/*msg_sequence*/, timestamp, calibration_payload);
            s.calibration_written = true;
        }

        const std::vector<std::byte> image_payload = encode_frame(
            _options.codec, 
            image, 
            s.color_format, 
            timestamp,
            /*coord_frame_id*/s.stream_name,
            _options.encode
        );
        this->_write_mcap_message(s.image_channel_id, s.next_image_sequence++, timestamp, image_payload);

        _frames_written.fetch_add(1, std::memory_order_relaxed);
        _payload_bytes.fetch_add(image_payload.size(), std::memory_order_relaxed);
        if (!_has_frames.load(std::memory_order_relaxed)) {
            _first_timestamp.store(timestamp, std::memory_order_relaxed);
            _has_frames.store(true, std::memory_order_relaxed);
        }
        _last_timestamp.store(timestamp, std::memory_order_relaxed);

        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("recording_writer::write_frame failed: {}", e.what());
        return false;
    }

    void recording_writer::_write_mcap_message(
        const uint16_t channel_id,
        const uint32_t msg_sequence,
        const hw::timestamp_t timestamp,
        const std::span<const std::byte> msg_payload)
    {
        const auto time = static_cast<mcap::Timestamp>(timestamp.time_since_epoch().count());

        mcap::Message mcap_msg{};
        mcap_msg.channelId = channel_id;
        mcap_msg.sequence = msg_sequence;
        mcap_msg.logTime = time;
        mcap_msg.publishTime = time;
        mcap_msg.dataSize = msg_payload.size();
        mcap_msg.data = msg_payload.data();

        if (const mcap::Status status = _writer->write(mcap_msg); !status.ok()) {
            throw std::runtime_error{ std::format("failed to write message: {}", status.message) };
        }
    }

    void recording_writer::close() noexcept
    {
        if (!_writer) { return; }

        _writer->close();
        _writer.reset();
        _streams.clear();

        const recording_stats_t final_stats = this->stats();
        spdlog::info("recording closed: {} ({} frames, {:.1f} s, {:.1f} MB)"
            , _path.string()
            , final_stats.frames_written
            , std::chrono::duration<double>{ final_stats.duration }.count()
            , static_cast<double>(final_stats.file_bytes) / (1024.0 * 1024.0)
        );
    }

    recording_stats_t recording_writer::stats() const noexcept
    {
        std::error_code ec;
        const uint64_t size = _path.empty() ? 0u : std::filesystem::file_size(_path, ec);

        const auto first = _first_timestamp.load(std::memory_order_relaxed);
        const auto last = _last_timestamp.load(std::memory_order_relaxed);
        const bool has_frames = _has_frames.load(std::memory_order_relaxed);

        return recording_stats_t{
            .frames_written = _frames_written.load(std::memory_order_relaxed),
            .payload_bytes = _payload_bytes.load(std::memory_order_relaxed),
            .file_bytes = ec ? 0u : size,
            .duration = has_frames ? (last - first) : std::chrono::nanoseconds{ 0 },
        };
    }

} // namespace io

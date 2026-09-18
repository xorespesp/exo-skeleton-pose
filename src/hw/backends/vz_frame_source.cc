#include "vz_frame_source.hh"

#include <spdlog/spdlog.h>

#include <chrono>
#include <format>
#include <initializer_list>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace hw
{
    namespace
    {
        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

        // How long one capture may take before the provider is handed nothing and loops. Long
        // enough for a slow trigger, short enough that `close()` is never stuck behind a grab.
        constexpr uint32_t kGrabTimeoutMs = 500;

        // How long an interface scan may take before the cameras on it are counted as absent.
        constexpr uint32_t kEnumerateTimeoutMs = 500;

        // Which of the camera's own illuminant presets the channel ratios come from.
        constexpr const char* kLightSourcePreset = "Daylight6500K";

        vz::frame_format_t to_vz_format(const frame_format_t format)
        {
            return (format == frame_format_t::gray8) ? vz::frame_format_t::gray
                                                     : vz::frame_format_t::bgr;
        }

        // Whether a GenICam pixel format name carries colour.
        bool pixel_format_carries_color(const std::string& name)
        {
            // Check monochrome 
            return !name.starts_with("Mono");
        }

        // GenICam nodes vary by model, so one this camera does not carry is a knob it does not have
        // and nothing that went wrong.
        void pin_enum_node_if_present(vz::device& device, const char* name, const char* value)
        {
            if (!device.read_enum(name).has_value()) { return; }
            if (!device.write_enum(name, value)) {
                spdlog::warn("vz: could not set {} to {}: {}", name, value, device.last_err_msg());
            }
        }

        void pin_bool_node_if_present(vz::device& device, const char* name, const bool value)
        {
            if (!device.read_bool(name).has_value()) { return; }
            if (!device.write_bool(name, value)) {
                spdlog::warn("vz: could not set {} to {}: {}", name, value, device.last_err_msg());
            }
        }

        // Settles the camera's colour processing, so every open starts from the same response.
        //
        // These nodes survive power cycles, so a value an earlier session left behind reaches this
        // one. Each goes to a state the camera itself names: the chroma from the illuminant preset,
        // everything else from its "do nothing" setting. Pinned at any frame format, since they also
        // reach the luminance a gray conversion produces.
        //
        // TODO: fixed here because no installation has asked for a state of its own. Should one,
        // these belong beside exposure and gain in the profile.
        void pin_persistent_color_response(vz::device& device)
        {
            // Auto rewrites the channel ratios frame by frame, which a fitted model cannot follow.
            // Off leaves them where they were last driven, so the preset states them right after.
            pin_enum_node_if_present(device, "BalanceWhiteAuto", "Off");
            pin_enum_node_if_present(device, "LightSourcePreset", kLightSourcePreset);

            // Gamma bends each channel by an amount that depends on how bright it already is, 
            // which moves a* and b* without moving the marker.
            pin_bool_node_if_present(device, "GammaEnable", false);

            // Saturation scales chroma outright; 
            // the luminance lookup and the digital shift reach it by another route.
            pin_enum_node_if_present(device, "SaturationMode", "Off");
            pin_bool_node_if_present(device, "LUTEnable", false);
            if (device.read_int("DigitalShift").has_value() && !device.write_int("DigitalShift", 0)) {
                spdlog::warn("vz: could not clear the digital shift: {}", device.last_err_msg());
            }

            // Not colour, but the same leftover: a mirrored readout turns the leg upside down in the
            // frame, and the colour markers are named by their order down it.
            pin_bool_node_if_present(device, "ReverseX", false);
            pin_bool_node_if_present(device, "ReverseY", false);

            // What is left unpinned, said out loud. `BlackLevel` is an offset added before
            // digitisation, so raising it compresses chroma toward neutral, and the camera names no
            // neutral value for it to be pinned to.
            std::string ratios;
            if (const std::optional<vz::enum_feature_t> selector = device.read_enum("BalanceRatioSelector"))
            {
                for (const std::string& channel : selector->entries)
                {
                    if (!device.write_enum("BalanceRatioSelector", channel)) { continue; }
                    const std::optional<vz::float_feature_t> r = device.read_float("BalanceRatio");
                    if (!r.has_value()) { continue; }
                    if (!ratios.empty()) { ratios += " "; }
                    ratios += std::format("{}={:.3f}", channel, r->value);
                }
            }
            const std::optional<vz::float_feature_t> black = device.read_float("BlackLevel");
            spdlog::info("vz: color response settled (preset {}, {}, black level {})"
                , kLightSourcePreset
                , ratios.empty() ? std::string{ "no balance ratios" } : ratios
                , black.has_value() ? std::format("{:.1f}", black->value) : std::string{ "n/a" });
        }

        // Cameras disagree on which of the interchangeable GenICam names they carry, 
        // so each is tried in turn.
        std::optional<int64_t> read_first_readable_int_node(
            const vz::device& device,
            std::initializer_list<const char*> names)
        {
            for (const char* name : names) {
                if (const std::optional<vz::int_feature_t> f = device.read_int(name)) {
                    return f->value;
                }
            }
            return std::nullopt;
        }
    } // namespace

    vz_frame_source::~vz_frame_source()
    {
        this->close();
    }

    bool vz_frame_source::open(const vz_device_config_t& config) noexcept try
    {
        std::scoped_lock lk{ _mtx };

        // SDK 는 open 시 디바이스를 항상 시리얼값 기반으로 구분하므로, 인덱스 모드에서는 해당하는 시리얼을 찾는다.
        const std::optional<std::string> serial = std::visit(overloaded{
            [](const device_serial_t& by_serial) -> std::optional<std::string> { return by_serial.value; },
            [](const device_index_t& by_index) -> std::optional<std::string>
            {
                std::string err_msg;
                const std::vector<vz::device_info_t> found = vz::device::enumerate(kEnumerateTimeoutMs, &err_msg);
                if (found.empty()) {
                    spdlog::error("vz: no camera found{}{}", err_msg.empty() ? "" : ": ", err_msg);
                    return std::nullopt;
                }
                if (by_index.value >= found.size()) {
                    spdlog::error("vz: camera #{} was asked for, but {} camera(s) are attached"
                        , by_index.value
                        , found.size()
                    );
                    return std::nullopt;
                }
                if (found.size() > 1) {
                    spdlog::warn("vz: opening camera #{} by its position among {} cameras; give a serial to pin it"
                        , by_index.value
                        , found.size()
                    );
                }
                return found[by_index.value].serial;
            },
        }, config.device_selector);
        if (!serial.has_value()) { return false; }

        if (!_device.open(*serial)) { return false; }
        _device_serial = *serial;

        _frame_format = config.frame_format;
        _device.set_frame_format(to_vz_format(_frame_format));

        // Colour classification needs the sensor to deliver colour. A monochrome format debayers to
        // R=G=B, which reads as a valid three-channel frame the whole way to the detector and
        // measures nothing, so the stream is refused here where the cause is still legible.
        if (_frame_format != frame_format_t::gray8)
        {
            if (const std::optional<vz::enum_feature_t> pf = _device.read_enum("PixelFormat");
                pf.has_value() && !pixel_format_carries_color(pf->value))
            {
                spdlog::error("vz: the camera streams '{}', which carries no color; set a color "
                              "pixel format on it before opening a color-marker profile", pf->value);
                _device.close();
                return false;
            }
        }

        pin_persistent_color_response(_device);

        // The calibration describes the sensor's whole readout, so the sensor extents are what
        // is wanted. `Width`/`Height` report the programmed window instead and come last, for
        // a camera that names no maximum.
        const std::optional<int64_t> sensor_w = read_first_readable_int_node(_device, { "WidthMax", "SensorWidth", "Width" });
        const std::optional<int64_t> sensor_h = read_first_readable_int_node(_device, { "HeightMax", "SensorHeight", "Height" });
        if (!sensor_w.has_value() || !sensor_h.has_value() || *sensor_w <= 0 || *sensor_h <= 0) {
            spdlog::error("vz: the camera reports no frame size");
            _device.close();
            return false;
        }
        const int width = static_cast<int>(*sensor_w);
        const int height = static_cast<int>(*sensor_h);

        // A readout window survives across opens, so one left from an earlier run would
        // silently narrow this one. Start whole and let `try_set_roi()` narrow it.
        if (!_device.set_roi(vz::roi_t{ .width = *sensor_w, .height = *sensor_h })) {
            spdlog::warn("vz: could not reset the readout window: {}", _device.last_err_msg());
        }

        // Left unset, the camera keeps whatever it was last given. A manual value only holds
        // with the matching auto mode off, which is a separate node: with it left on, the
        // camera overwrites what was just written on the next frame.
        if (config.exposure_us.has_value()) {
            if (!_device.write_enum("ExposureAuto", "Off")
                || !_device.write_float("ExposureTime", *config.exposure_us))
            {
                spdlog::warn("vz: could not set the exposure: {}", _device.last_err_msg());
            }
        }
        if (config.gain.has_value()) {
            if (!_device.write_enum("GainAuto", "Off")
                || !_device.write_float("Gain", *config.gain))
            {
                spdlog::warn("vz: could not set the gain: {}", _device.last_err_msg());
            }
        }

        // 프레임 레이트.
        // NOTE:
        //   이 노드는 전원이 켜져 있는 동안 지난 세션의 값을 들고 있으므로, 값이 없을 때도 제한을 명시적으로 풀어 카메라 상한으로 프리런하게 한다. 
        //   GenICam 은 켜고 끄는 노드를 두 가지 철자로 두므로, 카메라가 가진 쪽을 쓴다.
        if (config.frame_rate_fps.has_value()) {
            pin_enum_node_if_present(_device, "AcquisitionFrameRateMode", "On");
            pin_bool_node_if_present(_device, "AcquisitionFrameRateEnable", true);
            if (!_device.write_float("AcquisitionFrameRate", *config.frame_rate_fps)) {
                spdlog::warn("vz: could not set the frame rate: {}", _device.last_err_msg());
            }
        } else {
            pin_enum_node_if_present(_device, "AcquisitionFrameRateMode", "Off");
            pin_bool_node_if_present(_device, "AcquisitionFrameRateEnable", false);
        }
        if (const std::optional<vz::float_feature_t> rate = _device.read_float("AcquisitionFrameRate")) {
            spdlog::info("vz: frame rate {:.2f} fps (settable {:.2f}..{:.2f})", rate->value, rate->min, rate->max);
        }
        if (const std::optional<vz::float_feature_t> resulting = _device.read_float("ResultingFrameRate")) {
            spdlog::info("vz: resulting frame rate {:.2f} fps after exposure and bandwidth", resulting->value);
        }

        _calib = calibration_t{};
        _calib.frame_resolution = Eigen::Vector2i{ width, height };

        // The camera carries no intrinsics of its own, so they arrive measured off-line. A set
        // measured on a different frame size would project plausibly and wrongly, which is the
        // hardest failure to notice, so it is refused rather than rescaled.
        if (config.intrinsic.has_value()) {
            const intrinsic_t& intr = *config.intrinsic;
            if (intr.calib_resolution != _calib.frame_resolution) {
                spdlog::error("vz: the supplied intrinsics were measured on {}x{} but the sensor "
                              "is {}x{}; running without them"
                    , intr.calib_resolution.x()
                    , intr.calib_resolution.y()
                    , width
                    , height
                );
            }
            else {
                _calib.intrinsic = intr;
                if (config.distortion.has_value()) { _calib.distortion = *config.distortion; }
            }
        }

        spdlog::info("vz: camera '{}' ready ({}x{}, {}, intrinsics {})"
            , _device_serial
            , width
            , height
            , frame_format_to_str(_frame_format)
            , _calib.intrinsic.fx > 0.0f ? "supplied" : "absent"
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("vz_frame_source::open failed: {}", e.what());
        return false;
    }

    bool vz_frame_source::is_valid() const
    {
        std::scoped_lock lk{ _mtx };
        return _device.is_open();
    }

    stream_descriptor_t vz_frame_source::get_stream_descriptor() const
    {
        std::scoped_lock lk{ _mtx };
        return stream_descriptor_t{
            .sensor_backend = sensor_backend_t::vz,
            .device_serial = _device_serial,
        };
    }

    void vz_frame_source::close()
    {
        std::scoped_lock lk{ _mtx };
        _device.close();
    }

    std::optional<roi_t> vz_frame_source::try_set_roi(const roi_t& roi)
    {
        std::scoped_lock lk{ _mtx };
        if (!_device.is_open()) { return std::nullopt; }

        const roi_t fitted = clamp_roi(roi, _calib.frame_resolution.x(), _calib.frame_resolution.y());
        if (fitted.is_empty()) {
            spdlog::warn("vz: the requested ROI leaves nothing of the frame");
            return std::nullopt;
        }

        // The readout window is part of the acquisition setup and is locked while frames are
        // in flight, so a live stream steps aside for the write and picks up again after.
        const bool was_streaming = _device.is_streaming();
        if (was_streaming) { _device.stop_stream(); }

        const bool written = _device.set_roi(vz::roi_t{
            .width = fitted.width,
            .height = fitted.height,
            .offset_x = fitted.x,
            .offset_y = fitted.y,
        });
        if (!written) {
            spdlog::warn("vz: the camera refused the readout window: {}", _device.last_err_msg());
        }

        const std::optional<vz::roi_t> granted = _device.read_roi();
        if (was_streaming && !_device.start_stream()) {
            spdlog::error("vz: acquisition did not resume after the readout window was written");
        }
        if (!written || !granted.has_value()) { return std::nullopt; }

        return roi_t{
            .x = static_cast<int>(granted->offset_x),
            .y = static_cast<int>(granted->offset_y),
            .width = static_cast<int>(granted->width),
            .height = static_cast<int>(granted->height),
        };
    }

    std::optional<sensor_frameset> vz_frame_source::fetch_next_sensor_frameset()
    {
        std::scoped_lock lk{ _mtx };
        if (!_device.is_open()) { return std::nullopt; }

        // Acquisition is deferred to here, which leaves the readout window writable for as
        // long as nobody has asked for a frame.
        if (!_device.is_streaming() && !_device.start_stream()) {
            return std::nullopt;
        }

        // 이 캡처의 Unix 시각. 앵커가 서기 전의 캡처는 warmup 표본으로 쓰이고 비어 있는 값이 돌아온다.
        const auto resolve_capture_timestamp = [this](const vz::captured_frame_t& captured) -> std::optional<timestamp_t>
        {
            // 캡처 클럭이 없으면 도착 시각이 전부이고, 호스트 스케줄링이 거기 드러난다.
            if (!captured.device_timestamp.has_value()) {
                return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
            }

            if (_clock_warmup_samples < kClockWarmupSamples)
            {
                // 표본 중 offset 이 가장 작은 앵커가 남는다.
                const clock_anchor_t candidate{ *captured.device_timestamp };
                if (!_clock_anchor.has_value() || candidate.offset() < _clock_anchor->offset()) { _clock_anchor = candidate; }
                ++_clock_warmup_samples;
                return std::nullopt;
            }

            // 카메라 클럭은 매끄럽지만 epoch 이 제 것이라, 앵커가 프레임 간격은 건드리지 않고 Unix 시각으로 올린다.
            return _clock_anchor->to_unix(*captured.device_timestamp);
        };

        // 시각을 받은 캡처가 나올 때까지 받는다.
        std::optional<vz::captured_frame_t> captured;
        std::optional<timestamp_t> timestamp;
        do
        {
            captured = _device.grab_frame(kGrabTimeoutMs);
            if (!captured.has_value() || captured->image.empty()) { return std::nullopt; }

            timestamp = resolve_capture_timestamp(*captured);
        }
        while (!timestamp.has_value());

        return sensor_frameset{ std::make_shared<sensor_frame>(
            std::move(captured->image),
            _frame_format,
            *timestamp
        ) };
    }

} // namespace hw

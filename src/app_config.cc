#include "app_config.hh"

#include "utils/serializable.hh"

#include <nlohmann/json.hpp>

#include <concepts>
#include <format>
#include <fstream>
#include <limits>
#include <span>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#ifdef _WIN32
#  include <windows.h> // GetModuleFileNameW
#endif

namespace app
{
    namespace
    {
        using json = nlohmann::ordered_json;

        // Deep enough to clear a build tree's out/build/<preset>/, stopping at the filesystem root.
        constexpr int kSearchLevels = 6;

        // The directory holding the running executable, empty where it cannot be determined.
        // Anchored to the executable so the same paths come out however the app is launched.
        std::filesystem::path exe_dir()
        {
#ifdef _WIN32
            std::wstring buf(MAX_PATH, L'\0');
            for (;;) {
                const DWORD written = ::GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
                if (written == 0) { break; }
                if (written < buf.size()) {
                    return std::filesystem::path{ buf.substr(0, written) }.parent_path();
                }
                buf.resize(buf.size() * 2); // a filled buffer means the path was truncated
            }
#endif
            return {};
        }

        // First existing `<dir>/relative` at or above the executable, else nullopt.
        std::optional<std::filesystem::path> search_above_exe(const std::filesystem::path& relative)
        {
            const std::filesystem::path start = exe_dir();
            if (start.empty()) { return std::nullopt; }

            std::error_code ec;
            std::filesystem::path dir = start;
            for (int level = 0; level < kSearchLevels && dir.has_relative_path(); ++level) {
                if (const std::filesystem::path candidate = dir / relative;
                    std::filesystem::exists(candidate, ec))
                {
                    return candidate;
                }
                dir = dir.parent_path();
            }
            return std::nullopt;
        }

        // =====================================================================================
        // Leaf conversions
        //
        // One pair per type a field list mentions but that carries no list of its own, because it
        // is a single value in the file. Nothing here knows a key name.
        //
        // The keys themselves are not spelled here: every stored type names its own beside its
        // members with `DECLARE_SERIALIZABLE_FIELDS` (see `utils/serializable.hh`), so a field and
        // its key are added in one place. What this file owns is the format those lists are
        // written in.
        // =====================================================================================

        template <typename T> struct is_duration : std::false_type {};
        template <typename R, typename P> struct is_duration<std::chrono::duration<R, P>> : std::true_type {};

        template <typename T> struct is_optional : std::false_type {};
        template <typename T> struct is_optional<std::optional<T>> : std::true_type {};

        // 목록은 JSON 배열이고 원소마다 재귀한다. 원소의 꼴은 원소가 정한다.
        template <typename T> struct is_vector : std::false_type {};
        template <typename T, typename A> struct is_vector<std::vector<T, A>> : std::true_type {};

        json to_json(const auto& value);

        json to_json_leaf(const source_address& v)              { return json(v.to_string()); }
        json to_json_leaf(pose::camera_view_t v)                { return json(std::string{ pose::camera_view_name(v) }); }
        json to_json_leaf(pose::tag_detector::pose_method_t v)  { return json(std::string{ pose::pose_method_to_str(v) }); }
        json to_json_leaf(marker_kind_t v)                      { return json(std::string{ marker_kind_name(v) }); }
        json to_json_leaf(const Eigen::Vector2d& v)             { return json::array({ v.x(), v.y() }); }
        json to_json_leaf(const Eigen::Vector2i& v)             { return json::array({ v.x(), v.y() }); }

        // Row major, so the file reads the way the matrix is written.
        json to_json_leaf(const Eigen::Matrix2d& v)
        {
            return json::array({ json::array({ v(0, 0), v(0, 1) }),
                                 json::array({ v(1, 0), v(1, 1) }) });
        }

        json to_json(const auto& value)
        {
            using T = std::remove_cvref_t<decltype(value)>;
            if constexpr (is_optional<T>::value) {
                return value.has_value() ? to_json(*value) : json(nullptr);
            }
            else if constexpr (is_duration<T>::value) {
                return json(value.count());
            }
            else if constexpr (is_vector<T>::value) {
                json node = json::array();
                for (const auto& element : value) { node.push_back(to_json(element)); }
                return node;
            }
            else if constexpr (utils::has_serializable_fields<T>) {
                json node = json::object();
                auto writer = [&node](const char* key, const auto& field) { node[key] = to_json(field); };
                T::visit_serializable_fields(writer, value);
                return node;
            }
            else if constexpr (requires { to_json_leaf(value); }) {
                return to_json_leaf(value);
            }
            else {
                return json(value); // arithmetic, bool, std::string
            }
        }

        // -------------------------------------------------------------------------------------

        std::string key_name(std::string_view group, const char* key)
        {
            return group.empty() ? std::string{ key } : std::format("{}.{}", group, key);
        }

        bool from_json(const json& node, std::string_view name, auto& out, std::string& err);

        bool from_json_leaf(const json& node, std::string_view name, source_address& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = source_address::try_parse(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be k4a:sn:<serial>, vz:sn:<serial> "
                                  "or a recording path, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, pose::camera_view_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = pose::camera_view_from_name(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be frontal, sagittal_left or sagittal_right, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name,
            pose::tag_detector::pose_method_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = pose::pose_method_from_str(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be orthogonal_iteration or homography, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, marker_kind_t& out, std::string& err)
        {
            if (!node.is_string()) { err = std::format("'{}' must be a string", name); return false; }
            const auto text = node.get<std::string>();
            const auto parsed = marker_kind_from_name(text);
            if (!parsed.has_value()) {
                err = std::format("'{}' must be apriltag or color_marker, not '{}'", name, text);
                return false;
            }
            out = *parsed;
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Vector2d& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2) {
                err = std::format("'{}' must be an array of two numbers", name);
                return false;
            }
            try {
                out = Eigen::Vector2d{ node[0].get<double>(), node[1].get<double>() };
            }
            catch (const std::exception&) {
                err = std::format("'{}' holds something other than numbers", name);
                return false;
            }
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Vector2i& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2
                || !node[0].is_number_integer() || !node[1].is_number_integer())
            {
                err = std::format("'{}' must be an array of two whole numbers", name);
                return false;
            }
            out = Eigen::Vector2i{ node[0].get<int>(), node[1].get<int>() };
            return true;
        }

        bool from_json_leaf(const json& node, std::string_view name, Eigen::Matrix2d& out, std::string& err)
        {
            if (!node.is_array() || node.size() != 2
                || !node[0].is_array() || node[0].size() != 2
                || !node[1].is_array() || node[1].size() != 2)
            {
                err = std::format("'{}' must be two rows of two numbers", name);
                return false;
            }
            try {
                out << node[0][0].get<double>(), node[0][1].get<double>(),
                       node[1][0].get<double>(), node[1][1].get<double>();
            }
            catch (const std::exception&) {
                err = std::format("'{}' holds something other than numbers", name);
                return false;
            }
            return true;
        }

        // Reads `node` into `out`. `name` is the full dotted key, which every message here is
        // built around. An object is filled field by field, so a failure part-way leaves `out`
        // half-written; callers read into a scratch value and keep it only on success.
        bool from_json(const json& node, std::string_view name, auto& out, std::string& err)
        {
            using T = std::remove_cvref_t<decltype(out)>;

            if constexpr (is_optional<T>::value) {
                if (node.is_null()) { out.reset(); return true; }
                typename T::value_type value{};
                if (!from_json(node, name, value, err)) { return false; }
                out = std::move(value);
                return true;
            }
            else if constexpr (is_duration<T>::value) {
                if (!node.is_number()) { err = std::format("'{}' must be a number", name); return false; }
                const double value = node.get<double>();
                if (value < 0.0) { err = std::format("'{}' must not be negative", name); return false; }
                out = T{ value };
                return true;
            }
            else if constexpr (is_vector<T>::value) {
                if (!node.is_array()) { err = std::format("'{}' must be an array", name); return false; }
                T elements;
                elements.reserve(node.size());
                for (std::size_t i = 0; i < node.size(); ++i)
                {
                    typename T::value_type element{};
                    if (!from_json(node[i], std::format("{}[{}]", name, i), element, err)) { return false; }
                    elements.push_back(std::move(element));
                }
                out = std::move(elements);
                return true;
            }
            else if constexpr (utils::has_serializable_fields<T>) {
                if (!node.is_object()) { err = std::format("'{}' must be an object", name); return false; }
                bool ok = true;
                auto reader = [&](const char* key, auto& field) {
                    if (!ok) { return; }
                    // Every key of the field list has to be there. A default standing in for one
                    // the author left out is the failure this whole reader exists to prevent, and
                    // `--dump-config` prints a complete file to start from.
                    const auto it = node.find(key);
                    if (it == node.end()) {
                        err = std::format("'{}' is missing", key_name(name, key));
                        ok = false;
                        return;
                    }
                    // A field a list cannot write into is a stamp the file states and has to match,
                    // so it is read into a scratch value and compared. The load below is in the
                    // other arm because it would not compile against a const field.
                    using field_t = std::remove_reference_t<decltype(field)>;
                    if constexpr (std::is_const_v<field_t>) {
                        std::remove_const_t<field_t> stated{};
                        if (!from_json(*it, key_name(name, key), stated, err)) { ok = false; return; }
                        if (stated != field) {
                            err = std::format("'{}' is {}, and this build reads {}",
                                key_name(name, key), stated, field);
                            ok = false;
                        }
                    } else {
                        ok = from_json(*it, key_name(name, key), field, err);
                    }
                };
                T::visit_serializable_fields(reader, out);
                return ok; // keys the field list does not name are left alone
            }
            else if constexpr (requires { from_json_leaf(node, name, out, err); }) {
                return from_json_leaf(node, name, out, err);
            }
            else if constexpr (std::integral<T> && !std::same_as<T, bool>) {
                // Read wide and range-check: a number too big for the field would otherwise be
                // truncated into a plausible wrong value.
                if (!node.is_number_integer()) {
                    err = std::format("'{}' must be a whole number", name);
                    return false;
                }
                const int64_t value = node.get<int64_t>();
                if (value < static_cast<int64_t>(std::numeric_limits<T>::min())
                    || value > static_cast<int64_t>(std::numeric_limits<T>::max()))
                {
                    err = std::format("'{}' must be between {} and {}", name,
                        std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
                    return false;
                }
                out = static_cast<T>(value);
                return true;
            }
            else {
                try {
                    out = node.get<T>();
                    return true;
                }
                catch (const std::exception&) {
                    err = std::format("'{}' is not of the expected type", name);
                    return false;
                }
            }
        }

        // =====================================================================================
        // Semantic checks
        //
        // What the types alone cannot say. Reading happens first and answers "is this the right
        // shape"; this answers "is this a usable installation".
        // =====================================================================================

        // 카메라 엔트리 하나의 규칙. `path` 는 "cameras[i]".
        bool validate_camera(const camera_config_t& cam, const std::string& path, std::string& err)
        {
            if (cam.roi.has_value() && cam.roi->is_empty()) {
                err = std::format("'{}.roi' has no area", path);
                return false;
            }
            if (cam.frame_rate_fps.has_value() && *cam.frame_rate_fps <= 0.0) {
                err = std::format("'{}.frame_rate_fps' must be greater than zero, or null for no limit", path);
                return false;
            }

            // 주소의 텍스트 형태는 빈 시리얼을 되읽지 못한다. 그런 값이 파일에 적히면 그 파일은 로드되지 않는다.
            if (cam.source.is_device() && cam.source.device_serial().value.empty()) {
                err = std::format("'{}.source' names a camera with an empty serial", path);
                return false;
            }

            if (cam.source.is_recording())
            {
                // 파일이 그 스트림의 노출·게인·intrinsic·프레임 레이트를 나르므로 적을 자리가 아니다.
                if (cam.exposure_us.has_value() || cam.gain.has_value()) {
                    err = std::format("'{0}.exposure_us' and '{0}.gain' must both be null for a recording; "
                                      "the file carries what it was shot with", path);
                    return false;
                }
                if (!cam.intrinsics_file.empty()) {
                    err = std::format("'{}.intrinsics_file' must be empty for a recording; the file carries its calibration", path);
                    return false;
                }
                if (cam.frame_rate_fps.has_value()) {
                    err = std::format("'{}.frame_rate_fps' must be null for a recording; the file plays at its own rate", path);
                    return false;
                }
            }
            else if (cam.source.device_backend() == hw::sensor_backend_t::k4a)
            {
                if (!cam.intrinsics_file.empty()) {
                    err = std::format("'{}.intrinsics_file' must be empty for a K4A camera; the SDK reports its own", path);
                    return false;
                }
                if (cam.frame_rate_fps.has_value() && *cam.frame_rate_fps != 30.0) {
                    err = std::format("'{}.frame_rate_fps' must be null or 30 for a K4A camera; the SDK fixes it", path);
                    return false;
                }
            }
            return true;
        }

        // 색 캘리브레이션 하나의 규칙. `path` 는 "pose.detector.color_marker.calibration[i]".
        bool validate_color_calibration(const color_marker_calibration_t& calibration, const std::string& path, std::string& err)
        {
            const pose::color_marker_assigner::options_t& assigner = calibration.assigner;
            if (assigner.marker_diameter_m <= 0.0) {
                err = std::format("'{}.assigner.marker_diameter_m' must be greater than zero", path);
                return false;
            }
            if (assigner.search_radius_px <= 0.0) {
                err = std::format("'{}.assigner.search_radius_px' must be greater than zero", path);
                return false;
            }
            if (assigner.bone_length_tolerance <= 0.0 || assigner.bone_length_tolerance >= 1.0) {
                err = std::format("'{}.assigner.bone_length_tolerance' must be within (0, 1)", path);
                return false;
            }

            const pose::color_marker_detector::options_t& detector = calibration.detector;
            const pose::color_model_t& model = detector.model;
            const Eigen::Matrix2d& covariance = model.cov_ab;

            // A fitted colour is an ellipse on the a*b* plane, so its covariance has to describe
            // an area. A singular one inverts to nonsense and would admit every pixel or none.
            if (covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0 || covariance.determinant() <= 0.0) {
                err = std::format("'{}.detector.model.cov_ab' does not describe a spread "
                                  "(both diagonals and the determinant must be positive)", path);
                return false;
            }
            if (model.max_distance <= 0.0) {
                err = std::format("'{}.detector.model.max_distance' must be greater than zero", path);
                return false;
            }
            if (detector.min_fill <= 0.0 || detector.min_fill > 1.0) {
                err = std::format("'{}.detector.min_fill' must be within (0, 1]", path);
                return false;
            }
            if (detector.min_score < 0.0 || detector.min_score > 1.0) {
                err = std::format("'{}.detector.min_score' must be within [0, 1]", path);
                return false;
            }
            if (detector.max_aspect < 1.0) {
                err = std::format("'{}.detector.max_aspect' must be at least 1.0 (1.0 = a circle)", path);
                return false;
            }
            return true;
        }

        bool validate(const app_config_t& cfg, std::string& err)
        {
            if (cfg.pose.detector.apriltag.tag_size_m <= 0.0) {
                err = "'pose.detector.apriltag.tag_size_m' must be greater than zero";
                return false;
            }
            if (cfg.server.port == 0) {
                err = "'server.port' must not be zero";
                return false;
            }
            const pose::tag_detector::options_t& apriltag = cfg.pose.detector.apriltag.detector;
            if (apriltag.quad_decimate < 1.0f) {
                err = "'pose.detector.apriltag.detector.quad_decimate' must be at least 1.0 (1.0 = full resolution)";
                return false;
            }
            if (apriltag.num_iters < 1 || apriltag.num_threads < 1) {
                err = "'pose.detector.apriltag.detector.num_iters' and '...num_threads' must be at least 1";
                return false;
            }

            // ----- cameras: 엔트리 하나씩, 그리고 목록으로서 -----
            const std::vector<camera_config_t>& cameras = cfg.cameras;
            std::size_t live_count = 0;
            std::size_t recording_count = 0;
            for (std::size_t i = 0; i < cameras.size(); ++i)
            {
                const camera_config_t& cam = cameras[i];
                const std::string path = std::format("cameras[{}]", i);
                if (!validate_camera(cam, path, err)) { return false; }

                if (cam.source.is_recording())
                {
                    ++recording_count;
                    // 한 세션은 녹화 하나를 재생한다. 엔트리 i 는 그 파일의 스트림 i 다.
                    if (i > 0 && cameras[0].source.is_recording() && !(cam.source == cameras[0].source)) {
                        err = std::format("'{}.source' names another recording than 'cameras[0].source'; "
                                          "every entry of a playback names the same file", path);
                        return false;
                    }
                }
                else
                {
                    ++live_count;
                    for (std::size_t j = 0; j < i; ++j) {
                        if (cameras[j].source == cam.source) {
                            err = std::format("'{}.source' names the same camera as 'cameras[{}].source'", path, j);
                            return false;
                        }
                    }
                }
            }
            if (live_count > 0 && recording_count > 0) {
                err = "'cameras' mixes a recording with live cameras; a session plays one recording or opens cameras";
                return false;
            }

            // ----- views: 엔트리들이 한 평면에 합의하고, 그 평면이 요구하는 수와 쪽을 갖춘다 -----
            const std::optional<pose::view_plane_t> plane = view_plane_of(cameras);
            if (!cameras.empty())
            {
                if (!plane.has_value()) {
                    err = "'cameras' must agree on one plane: every 'view' frontal, or every one sagittal";
                    return false;
                }
                if (*plane == pose::view_plane_t::frontal && cameras.size() != 1) {
                    err = std::format("a frontal setup has exactly one camera, and 'cameras' names {}", cameras.size());
                    return false;
                }
                if (*plane == pose::view_plane_t::sagittal)
                {
                    if (cameras.size() != 2) {
                        err = std::format("a sagittal setup has exactly two cameras (one sagittal_left, one "
                                          "sagittal_right), and 'cameras' names {}", cameras.size());
                        return false;
                    }
                    if (cameras[0].view == cameras[1].view) {
                        err = "'cameras[0].view' and 'cameras[1].view' name the same side; a sagittal setup "
                              "needs one sagittal_left and one sagittal_right";
                        return false;
                    }
                }
            }

            // ----- sync: 라이브 카메라 둘 이상을 묶을 때만 있고, 그때는 있어야 한다 -----
            if (live_count >= 2)
            {
                if (!cfg.sync.has_value()) {
                    err = "'sync' is required when two or more live cameras are opened together";
                    return false;
                }
                if (cfg.sync->reference_stream_idx >= cameras.size()) {
                    err = std::format("'sync.reference_stream_idx' must be below {}, the number of cameras", cameras.size());
                    return false;
                }
                if (cfg.sync->max_pair_skew.has_value() && cfg.sync->max_pair_skew->count() <= 0.0) {
                    err = "'sync.max_pair_skew_ms' must be greater than zero, or null to derive it from the frame interval";
                    return false;
                }

                // 프로토콜과 상태 보고가 스트림 하나의 백엔드·프레임 크기로 전체를 말하므로, 한 번에
                // 여는 카메라는 같은 기종이어야 한다.
                for (std::size_t i = 1; i < cameras.size(); ++i)
                {
                    if (cameras[i].source.device_backend() != cameras[0].source.device_backend()) {
                        err = std::format("'cameras[{}].source' names a {} camera and 'cameras[0].source' a {}; "
                                          "cameras opened together are one model", i,
                            hw::sensor_backend_to_str(cameras[i].source.device_backend()),
                            hw::sensor_backend_to_str(cameras[0].source.device_backend()));
                        return false;
                    }
                }

                // 한 묶음으로 나가려면 스트림들이 같은 간격으로 도착해야 한다. 적힌 값이 아니라 실제로
                // 도는 속도를 비교한다: K4A 는 SDK 가 30 으로 고정하고, VZ 는 값을 적지 않으면 자기
                // 상한으로 프리런한다. 전부 프리런이면 속도를 알 수 없을 뿐 구성은 성립하므로, 그때는
                // 여는 쪽이 경고한다.
                const auto effective_rate = [](const camera_config_t& cam) -> std::optional<double> {
                    if (cam.source.device_backend() == hw::sensor_backend_t::k4a) { return 30.0; }
                    return cam.frame_rate_fps;
                };
                const std::optional<double> first_rate = effective_rate(cameras[0]);
                for (std::size_t i = 1; i < cameras.size(); ++i)
                {
                    const std::optional<double> rate = effective_rate(cameras[i]);
                    if (rate.has_value() != first_rate.has_value()) {
                        err = std::format("'cameras[{}].frame_rate_fps' and 'cameras[0].frame_rate_fps' disagree on "
                                          "whether to pin the rate; cameras opened together run at one rate", i);
                        return false;
                    }
                    if (rate.has_value() && *rate != *first_rate) {
                        err = std::format("'cameras[{}]' runs at {:.1f} fps and 'cameras[0]' at {:.1f}; "
                                          "cameras opened together run at one rate", i, *rate, *first_rate);
                        return false;
                    }
                }
            }
            else if (cfg.sync.has_value())
            {
                err = "'sync' is only read when two or more live cameras are opened together; set it to null";
                return false;
            }

            // ----- colour markers -----
            const color_marker_config_t& color_marker = cfg.pose.detector.color_marker;

            // Colour markers are read off the image plane, which is the sagittal estimator's input.
            // A frontal run needs a marker whose distance can be solved, and a plain disc is not one.
            if (cfg.pose.detector.kind == marker_kind_t::color_marker
                && plane.has_value() && *plane != pose::view_plane_t::sagittal)
            {
                err = "'pose.detector.kind' color_marker requires sagittal cameras (a plain disc solves no distance)";
                return false;
            }

            // A colour model is a fixed pair of numbers on the a*b* plane, and a camera free to
            // choose its own exposure or gain moves a marker off them: both scale luminance, and
            // a* and b* follow its cube root. Left on auto that happens mid-run, with nothing to
            // signal it but markers that stop being found, so the pairing is refused up front.
            //
            // Only for a camera this config opens. A recording carries the frames it was shot
            // with, so there is nothing to program into it and no brightness left to drift.
            if (cfg.pose.detector.kind == marker_kind_t::color_marker)
            {
                for (std::size_t i = 0; i < cameras.size(); ++i)
                {
                    const camera_config_t& cam = cameras[i];
                    if (cam.source.is_recording()) { continue; }
                    if (!cam.exposure_us.has_value() || !cam.gain.has_value()) {
                        err = std::format("'pose.detector.kind' color_marker requires 'cameras[{0}].exposure_us' and "
                                          "'cameras[{0}].gain' to name values (a colour model cannot follow auto)", i);
                        return false;
                    }
                }
            }

            // An absent calibration is not an error: it is what a profile says before anyone has
            // measured that camera. The detector then finds nothing and says so. One that is
            // present has to describe something, since every number in it is a ratio of one
            // measured quantity to another and each has a range it cannot mean anything outside of.
            //
            // 길이는 색 실행에만 건다. 파이프라인이 스트림마다 `calibration[stream_idx]` 를 집으므로
            // 그때는 카메라 수와 같아야 하고, 다른 마커 종류는 이 블록을 읽지 않는다.
            if (cfg.pose.detector.kind == marker_kind_t::color_marker
                && color_marker.calibration.size() != cameras.size())
            {
                err = std::format("'pose.detector.color_marker.calibration' has {} entr{}, one per camera is "
                                  "needed ({}); null stands for a camera not yet measured",
                    color_marker.calibration.size(), color_marker.calibration.size() == 1 ? "y" : "ies", cameras.size());
                return false;
            }
            for (std::size_t i = 0; i < color_marker.calibration.size(); ++i)
            {
                if (!color_marker.calibration[i].has_value()) { continue; }
                if (!validate_color_calibration(*color_marker.calibration[i],
                        std::format("pose.detector.color_marker.calibration[{}]", i), err)) { return false; }
            }

            // The frame interval is clamped into [dt_min, dt_max] and a One Euro step divides by
            // it and by the cutoffs, so a zero or inverted value here yields NaN angles rather
            // than worse ones. Both estimators carry the same fields, so one check reads either.
            const auto validate_estimator = [&err](const char* path, const auto& opt) {
                if (opt.position_filter.min_cutoff_hz <= 0.0 || opt.position_filter.dcutoff_hz <= 0.0) {
                    err = std::format("'{0}.position_filter.min_cutoff_hz' and "
                                      "'{0}.position_filter.dcutoff_hz' must be greater than zero", path);
                    return false;
                }
                if (opt.position_filter.beta < 0.0) {
                    err = std::format("'{}.position_filter.beta' must not be negative", path);
                    return false;
                }
                if (opt.dt_min.count() <= 0.0) {
                    err = std::format("'{}.dt_min_s' must be greater than zero", path);
                    return false;
                }
                if (opt.dt_max < opt.dt_min) {
                    err = std::format("'{0}.dt_max_s' must be at least '{0}.dt_min_s'", path);
                    return false;
                }
                if (opt.max_hold.count() < 0.0 || opt.reset_gap.count() < 0.0) {
                    err = std::format("'{0}.max_hold_ms' and '{0}.reset_gap_ms' must not be negative", path);
                    return false;
                }
                return true;
            };
            if (!validate_estimator("pose.estimator.frontal", cfg.pose.estimator.frontal)) { return false; }
            if (!validate_estimator("pose.estimator.sagittal", cfg.pose.estimator.sagittal)) { return false; }

            return true;
        }

    } // namespace

    bool validate_config(const app_config_t& config, std::string& err)
    {
        return validate(config, err);
    }

    std::filesystem::path project_dir(std::string_view name)
    {
        if (const auto found = search_above_exe(std::filesystem::path{ name })) {
            return found->lexically_normal();
        }

        // No such folder above the executable, so its own directory stands in and output lands flat beside the binary.
        const std::filesystem::path dir = exe_dir();
        return dir.empty() ? std::filesystem::current_path() : dir;
    }

    bool load_config(const std::filesystem::path& path, app_config_t& out, std::string& err)
    {
        std::ifstream in{ path };
        if (!in) {
            err = std::format("cannot open '{}'", path.string());
            return false;
        }

        json root;
        try {
            root = json::parse(in, nullptr /*cb*/, true /*allow_exceptions*/, true /*ignore_comments*/);
        }
        catch (const std::exception& e) {
            err = e.what(); // nlohmann names the byte offset and what it expected there
            return false;
        }

        if (!root.is_object()) {
            err = "the document must be an object";
            return false;
        }

        // Built on a fresh value, so a failure part-way leaves the caller's config untouched.
        app_config_t cfg;
        if (!from_json(root, "", cfg, err)) { return false; }
        if (!validate_config(cfg, err)) { return false; }

        // A colour block that parsed and passed validation is a colour that was fitted, which is
        // what `valid` means to the detector. The file does not say it: a flag beside the numbers
        // could claim otherwise about them.
        for (auto& calibration : cfg.pose.detector.color_marker.calibration) {
            if (calibration.has_value()) { calibration->detector.model.valid = true; }
        }

        // A companion file is named relative to the profile that names it, so a profile folder can
        // be moved or copied whole.
        for (camera_config_t& cam : cfg.cameras)
        {
            if (cam.intrinsics_file.empty()) { continue; }
            const std::filesystem::path p{ cam.intrinsics_file };
            cam.intrinsics_file = (p.is_absolute() ? p : path.parent_path() / p).lexically_normal().string();
        }

        out = std::move(cfg);
        return true;
    }

    std::optional<pose::view_plane_t> view_plane_of(const std::span<const camera_config_t> cameras)
    {
        if (cameras.empty()) { return std::nullopt; }
        const pose::view_plane_t plane = pose::view_plane_of(cameras.front().view);
        for (const camera_config_t& cam : cameras) {
            if (pose::view_plane_of(cam.view) != plane) { return std::nullopt; }
        }
        return plane;
    }

    std::string dump_config(const app_config_t& config)
    {
        return to_json(config).dump(2);
    }

    bool save_config(const app_config_t& config, const std::filesystem::path& path, std::string& err)
    {
        // 리더가 거부할 파일은 쓰지 않는다. 쓰고 나서야 못 읽는 것을 아는 것이 이 검사가 막는 실패다.
        if (!validate_config(config, err)) { return false; }

        std::error_code ec;
        if (path.has_parent_path()) { std::filesystem::create_directories(path.parent_path(), ec); }

        std::ofstream out{ path, std::ios::trunc };
        if (!out) {
            err = std::format("cannot write '{}'", path.string());
            return false;
        }

        out << dump_config(config) << '\n';
        if (!out) {
            err = std::format("failed while writing '{}'", path.string());
            return false;
        }
        return true;
    }

    std::filesystem::path find_config_file(std::string_view name)
    {
        const std::filesystem::path spelled{ name };
        const std::filesystem::path path = spelled.has_parent_path()
            ? spelled
            : project_dir("configs") / spelled;

        std::error_code ec;
        return std::filesystem::exists(path, ec) ? path : std::filesystem::path{};
    }

} // namespace app

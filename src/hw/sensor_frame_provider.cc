#include "sensor_frame_provider.hh"

#include "backends/k4a_frame_source.hh"
#ifdef EXO_HAS_VZ_BACKEND
#include "backends/vz_frame_source.hh"
#endif

#include "io/mcap_record_player.hh"

#include <spdlog/spdlog.h>

#include <atomic>
#include <condition_variable>
#include <exception>
#include <format>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace hw
{
    namespace
    {
        constexpr auto kIdlePollSleep = std::chrono::milliseconds{ 5 };
        constexpr float kFpsEmaAlpha = 0.1f; // 최신 표본의 가중치

        template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

        std::string describe_roi(const std::optional<roi_t>& roi)
        {
            return roi.has_value()
                ? std::format("roi {}x{}+{}+{}", roi->width, roi->height, roi->x, roi->y)
                : std::string{ "whole frame" };
        }

        // 카메라 백엔드를 만드는 유일한 자리. 열지 못하면 null. 녹화 config 도 null 이다: 녹화는 `open()` 이
        // `io::mcap_record_player` 로 연다.
        std::shared_ptr<sensor_frame_source> make_camera_source(const source_config_t& config)
        {
            return std::visit(overloaded{
                [](const k4a_device_config_t& c) -> std::shared_ptr<sensor_frame_source> {
                    auto s = std::make_shared<k4a_device_capturer>();
                    if (!s->open(c)) { return nullptr; }
                    return s;
                },
                [](const vz_device_config_t& c) -> std::shared_ptr<sensor_frame_source> {
#ifdef EXO_HAS_VZ_BACKEND
                    auto s = std::make_shared<vz_frame_source>();
                    if (!s->open(c)) { return nullptr; }
                    return s;
#else
                    (void)c;
                    spdlog::error("provider: this build carries no VZ camera backend");
                    return nullptr;
#endif
                },
                [](const recording_config_t&) -> std::shared_ptr<sensor_frame_source> {
                    return nullptr;
                },
            }, config);
        }

        // 두 config 가 같은 카메라를 가리키는지. `describe()` 가 백엔드와 장치 선택자를 그대로 담으므로
        // 라벨이 같으면 같은 장치다.
        bool names_same_device(const source_config_t& a, const source_config_t& b)
        {
            return a.index() == b.index() && describe(a) == describe(b);
        }

        // K4A 유선 싱크의 master 는 열기가 곧 시작이라 subordinate 들 뒤에 열어야 한다.
        bool is_wired_sync_master(const source_config_t& config)
        {
            const auto* k4a = std::get_if<k4a_device_config_t>(&config);
            return k4a && k4a->wired_sync_mode == k4a_sync_role_t::master;
        }
    }

    // 구현 전부. `sensor_frame_provider` 의 메서드는 여기로 넘기기만 한다.
    struct sensor_frame_provider::impl final
    {
        // 올려 둔 seek. 수행은 폴링 스레드가 한다.
        struct seek_request_t
        {
            enum class kind_t { begin, end, timeline };
            kind_t kind{ kind_t::begin };
            timestamp_t at{}; // `timeline` 만
        };

        // 올려 둔 ROI. 요청이 없는 것과 전체 프레임으로 되돌리는 요청을 구분하려고 optional 을 한 번 더 감쌌다.
        struct roi_request_t
        {
            std::optional<roi_t> window; // 비어 있음: 전체 프레임
        };

        // 스트림 하나의 픽셀 프레임. 네 값이 한 락 아래에서 함께 바뀌므로 읽는 쪽은 언제나 같은 윈도우의
        // 값을 본다.
        struct stream_geometry_t
        {
            calibration_t calib{};
            Eigen::Vector2i frame_resolution{ Eigen::Vector2i::Zero() };
            Eigen::Vector2i full_frame_resolution{ Eigen::Vector2i::Zero() };
            std::optional<roi_t> roi; // 걸려 있는 ROI
        };

        // 소스를 연 뒤 스트림 하나가 전달한 것.
        struct stream_delivery_stats_t
        {
            float    update_rate_fps{ 0.0f }; // EMA 로 평활
            uint64_t frames_delivered{ 0 };
        };

        // 스트림이 무엇인지. 열 때 소스에서 읽어 고정하므로, 폴링 스레드의 블로킹 fetch 와 무관하게 읽힌다.
        struct stream_info_t
        {
            frame_format_t frame_format{};
            stream_descriptor_t descriptor{};
        };

        ~impl();

        void add_observer(std::shared_ptr<synced_frameset_observer> observer);
        void remove_observer(const std::shared_ptr<synced_frameset_observer>& observer);
        bool is_opened() const;
        bool open(const source_config_t& config) noexcept;
        bool open_synced(std::span<const source_config_t> member_configs, const sync_options_t& sync_options) noexcept;
        void close();
        std::size_t stream_count() const;
        calibration_t get_calibration(std::size_t stream_idx) const;
        frame_format_t get_frame_format(std::size_t stream_idx) const;
        stream_descriptor_t get_stream_descriptor(std::size_t stream_idx) const;
        Eigen::Vector2i get_frame_resolution(std::size_t stream_idx) const;
        Eigen::Vector2i get_full_frame_resolution(std::size_t stream_idx) const;
        std::optional<roi_t> get_effective_roi(std::size_t stream_idx) const;
        void set_roi(std::size_t stream_idx, const std::optional<roi_t>& roi);
        float get_current_update_rate(std::size_t stream_idx) const;
        uint64_t get_frames_delivered(std::size_t stream_idx) const;
        sync_stats_t get_sync_stats() const;
        void play();
        void pause();
        void seek_recording_to_begin();
        void seek_recording_to_end();
        void seek_recording_timeline(timestamp_t timestamp);
        std::chrono::nanoseconds get_recording_length() const;
        timestamp_t get_first_record_timestamp() const;
        timestamp_t get_last_record_timestamp() const;
        void set_update_speed(float factor);

        // `player` 는 녹화일 때만 non-null 이다. `requested_rois` 는 스트림마다 하나.
        void _install_source(
            std::unique_ptr<frameset_synchronizer> synchronizer,
            std::shared_ptr<record_player_source> player,
            std::string source_name,
            const std::vector<std::optional<roi_t>>& requested_rois
        );

        // 프레임 기하의 유일한 writer. 한 번에 한 스트림을 쓴다. 바뀌었음을 관찰자에게 알리는 것은
        // 호출자의 몫이다.
        void _install_frame_geometry(
            frameset_synchronizer& synchronizer,
            std::size_t stream_idx,
            const std::optional<roi_t>& requested_roi
        );

        // 올라온 ROI 들을 수행한다. 픽셀 프레임이 실제로 움직인 스트림들을 돌려준다. 소스가 거부했거나
        // 이미 걸린 값에 스냅한 요청은 들지 않는다.
        std::vector<std::size_t> _apply_pending_rois();

        bool _has_pending_roi_request() const; // `_wake_cv_mtx` 아래에서 부른다

        void _start_thread();
        void _stop_thread();
        void _polling_thread_proc();
        void _end_stream_on_error(std::string_view reason); // 폴링 스레드의 마지막 행동

        // seek 을 폴링 스레드 몫으로 올려 둔다. 어느 스레드에서든 부를 수 있고, 반복 재생이 끝에서 처음으로
        // 되돌릴 때도 같은 길을 쓴다.
        void _post_seek_request(const seek_request_t& request);

        std::vector<std::shared_ptr<synced_frameset_observer>> _snapshot_observers() const;
        void _notify_synced_frameset_update(const synced_frameset& frameset);
        void _notify_sensor_stream_reset();
        void _notify_sensor_frame_geometry_changed(std::size_t stream_idx);
        void _notify_sensor_stream_end(stream_end_reason_t reason);

        // 폴링 스레드가 fetch 하는 synchronizer 와, 녹화일 때만 있는 player(seek 과 길이 조회).
        // 둘은 스레드를 띄우기 전에 놓이고 join 한 뒤에 치워지므로 폴링 스레드는 락 없이 쓴다.
        // `_source_mtx` 는 바깥 getter 와 그 교체 지점 사이만 지키고, 그래서 getter 는 블로킹 fetch 뒤에
        // 서지 않는다.
        std::unique_ptr<frameset_synchronizer> _synchronizer;
        std::shared_ptr<record_player_source> _player;
        mutable std::mutex _source_mtx;

        std::thread _thread;
        std::atomic<bool> _running{ false };
        std::atomic<bool> _paused{ false };
        std::atomic<bool> _auto_repeat{ true }; // 녹화가 끝에서 처음부터 다시 돈다
        std::atomic<bool> _need_repace{ false }; // 재생 페이싱 앵커를 다시 잡으라는 요청

        // 수행을 기다리는 seek 과 ROI, 그리고 폴링 스레드를 sleep 에서 깨우는 신호. seek 은 가장 새것이
        // 남고, ROI 는 스트림마다 가장 새것이 남는다.
        std::optional<seek_request_t> _pending_seek_req;
        std::vector<std::optional<roi_request_t>> _pending_roi_reqs; // 스트림 인덱스로 찾는다
        std::condition_variable _wake_cv;
        mutable std::mutex _wake_cv_mtx;

        std::vector<std::shared_ptr<synced_frameset_observer>> _observers;
        mutable std::mutex _observers_mtx;

        // `_install_frame_geometry()` 만 쓴다: 열 때, 그리고 ROI 마다. 읽기와 쓰기가 이 락 아래다.
        mutable std::mutex _geometry_mtx;
        std::vector<stream_geometry_t> _stream_geometries;

        // 폴링 스레드가 frameset 마다 한 번 쓰고, 어디서든 읽는다.
        mutable std::mutex _stream_stats_mtx;
        std::vector<stream_delivery_stats_t> _stream_stats;

        // 소스가 살아 있는 동안 고정. `_geometry_mtx` 아래에서 기하와 함께 설치된다.
        std::vector<stream_info_t> _stream_infos;

        // open 에서 정해지고 스트리밍 중에는 불변.
        std::string _source_name;

        std::atomic<uint32_t> _frameset_seq{ 0 };
        std::atomic<float> _frameset_rate{ 0.0f };
        std::atomic<float> _speed{ 1.0f };
    };

    sensor_frame_provider::impl::~impl()
    {
        this->close();
    }

    void sensor_frame_provider::impl::add_observer(std::shared_ptr<synced_frameset_observer> observer)
    {
        if (!observer) { return; }
        std::scoped_lock lk{ _observers_mtx };
        _observers.push_back(std::move(observer));
    }

    void sensor_frame_provider::impl::remove_observer(const std::shared_ptr<synced_frameset_observer>& observer)
    {
        std::scoped_lock lk{ _observers_mtx };
        std::erase(_observers, observer);
    }

    bool sensor_frame_provider::impl::is_opened() const
    {
        std::scoped_lock lk{ _source_mtx };
        return static_cast<bool>(_synchronizer);
    }

    bool sensor_frame_provider::impl::open(const source_config_t& config) noexcept try
    {
        this->close();

        std::unique_ptr<frameset_synchronizer> synchronizer;
        std::shared_ptr<record_player_source> player;

        if (const auto* recording = std::get_if<recording_config_t>(&config))
        {
            const std::shared_ptr<io::mcap_record_player> mcap_player = io::mcap_record_player::create();
            if (!mcap_player->open(recording->file)) { return false; }

            // 파일의 스트림들은 한 시간축을 공유하므로 기본 싱크 옵션으로 묶인다. 재생은 한 장도 건너뛰지
            // 않도록 링을 순서대로 비운다.
            player = mcap_player;
            const auto recording_streams = player->get_recording_streams();
            synchronizer = std::make_unique<frameset_synchronizer>(
                std::vector<std::shared_ptr<sensor_frame_source>>{ recording_streams.begin(), recording_streams.end() },
                sync_options_t{ .ring_policy = ring_policy_t::in_order });
        }
        else
        {
            std::shared_ptr<sensor_frame_source> camera = make_camera_source(config);
            if (!camera) { return false; }
            synchronizer = std::make_unique<frameset_synchronizer>(std::move(camera), ring_policy_t::newest_first);
        }

        // config 의 윈도우 하나가 소스의 모든 스트림에 걸린다.
        const std::optional<roi_t> requested_roi = std::visit(
            [](const auto& c) { return c.roi; },
            config
        );
        const std::vector<std::optional<roi_t>> requested_rois(synchronizer->stream_count(), requested_roi);

        this->_install_source(
            std::move(synchronizer),
            std::move(player),
            describe(config),
            requested_rois
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("provider: failed to open {}: {}", describe(config), e.what());
        return false;
    }

    bool sensor_frame_provider::impl::open_synced(
        const std::span<const source_config_t> member_configs,
        const sync_options_t& sync_options) noexcept try
    {
        this->close();

        if (member_configs.size() < 2) {
            spdlog::error("provider: a synced group needs at least two cameras, {} given", member_configs.size());
            return false;
        }
        if (sync_options.reference_stream_idx >= member_configs.size()) {
            spdlog::error("provider: reference stream {} is not among the {} members"
                , sync_options.reference_stream_idx, member_configs.size());
            return false;
        }
        for (const source_config_t& config : member_configs) {
            if (std::holds_alternative<recording_config_t>(config)) {
                spdlog::error("provider: a recording cannot join a synced group; its streams are already on one clock");
                return false;
            }
        }
        for (std::size_t a = 0; a < member_configs.size(); ++a) {
            for (std::size_t b = a + 1; b < member_configs.size(); ++b) {
                if (names_same_device(member_configs[a], member_configs[b])) {
                    spdlog::error("provider: members {} and {} both name {}", a, b, describe(member_configs[a]));
                    return false;
                }
            }
        }

        // 스트림 인덱스는 멤버 순서다. 여는 순서만 master 를 뒤로 미룬다.
        std::vector<std::size_t> open_order;
        for (std::size_t member_idx = 0; member_idx < member_configs.size(); ++member_idx) {
            if (!is_wired_sync_master(member_configs[member_idx])) { open_order.push_back(member_idx); }
        }
        for (std::size_t member_idx = 0; member_idx < member_configs.size(); ++member_idx) {
            if (is_wired_sync_master(member_configs[member_idx])) { open_order.push_back(member_idx); }
        }

        // 하나라도 실패하면 이 벡터가 사라지며 이미 연 카메라들을 닫는다.
        std::vector<std::shared_ptr<sensor_frame_source>> cameras(member_configs.size());
        for (const std::size_t member_idx : open_order)
        {
            cameras[member_idx] = make_camera_source(member_configs[member_idx]);
            if (!cameras[member_idx]) {
                spdlog::error("provider: member {} ({}) did not open; closing the others"
                    , member_idx, describe(member_configs[member_idx]));
                return false;
            }
        }

        std::vector<std::optional<roi_t>> requested_rois;
        std::string source_name;
        for (const source_config_t& config : member_configs)
        {
            requested_rois.push_back(std::visit([](const auto& c) { return c.roi; }, config));
            if (!source_name.empty()) { source_name += " + "; }
            source_name += describe(config);
        }

        this->_install_source(
            std::make_unique<frameset_synchronizer>(std::move(cameras), sync_options),
            nullptr,
            std::move(source_name),
            requested_rois
        );
        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::error("provider: failed to open a synced group: {}", e.what());
        return false;
    }

    void sensor_frame_provider::impl::_install_source(
        std::unique_ptr<frameset_synchronizer> synchronizer,
        std::shared_ptr<record_player_source> player,
        std::string source_name,
        const std::vector<std::optional<roi_t>>& requested_rois)
    {
        const std::size_t stream_count = synchronizer->stream_count();

        // 인덱스로 닿기 전에 스트림 수만큼 자리를 잡는다.
        {
            std::scoped_lock lk{ _geometry_mtx };
            _stream_geometries.assign(stream_count, stream_geometry_t{});
        }
        {
            std::scoped_lock lk{ _stream_stats_mtx };
            _stream_stats.assign(stream_count, stream_delivery_stats_t{});
        }

        std::vector<stream_info_t> stream_infos;
        stream_infos.reserve(stream_count);
        for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx)
        {
            const std::optional<roi_t> requested_roi =
                stream_idx < requested_rois.size() ? requested_rois[stream_idx] : std::nullopt;
            this->_install_frame_geometry(*synchronizer, stream_idx, requested_roi);

            stream_infos.push_back(stream_info_t{
                .frame_format = synchronizer->get_frame_format(stream_idx),
                .descriptor = synchronizer->get_stream_descriptor(stream_idx),
            });
        }
        {
            std::scoped_lock lk{ _geometry_mtx };
            _stream_infos = std::move(stream_infos);
        }

        _source_name = std::move(source_name);

        _frameset_seq.store(0);
        _frameset_rate.store(0.0f);
        _paused.store(false);
        _need_repace.store(true);
        {
            // 이전 소스를 향해 올린 요청은 버린다.
            std::scoped_lock lk{ _wake_cv_mtx };
            _pending_seek_req.reset();
            _pending_roi_reqs.assign(stream_count, std::nullopt);
        }

        {
            std::scoped_lock lk{ _source_mtx };
            _player = std::move(player);
            _synchronizer = std::move(synchronizer);
        }

        this->_notify_sensor_stream_reset();

        try
        {
            this->_start_thread();
        }
        catch (...)
        {
            _running.store(false);
            {
                std::scoped_lock lk{ _source_mtx };
                _synchronizer.reset();
                _player.reset();
            }
            throw;
        }

        spdlog::info("provider opened: {} ({} stream(s))", _source_name, stream_count);
        for (std::size_t stream_idx = 0; stream_idx < stream_count; ++stream_idx)
        {
            const stream_descriptor_t descriptor = this->get_stream_descriptor(stream_idx);
            spdlog::info("provider stream {}: {} '{}', {}, {}"
                , stream_idx
                , sensor_backend_to_str(descriptor.sensor_backend)
                , descriptor.device_serial
                , frame_format_to_str(this->get_frame_format(stream_idx))
                , describe_roi(this->get_effective_roi(stream_idx))
            );
        }
    }

    void sensor_frame_provider::impl::_install_frame_geometry(
        frameset_synchronizer& synchronizer,
        const std::size_t stream_idx,
        const std::optional<roi_t>& requested_roi)
    {
        // 전체 프레임의 캘리브레이션에서 시작하므로 주점은 한 번만 옮겨진다.
        calibration_t calib = synchronizer.get_calibration(stream_idx);
        const Eigen::Vector2i full = calib.frame_resolution;
        std::optional<roi_t> granted;

        if (requested_roi.has_value())
        {
            granted = synchronizer.try_set_roi(stream_idx, *requested_roi);

            if (!granted.has_value())
            {
                spdlog::warn("provider: ROI {}x{}+{}+{} was not applied to stream {}'s {}x{} frame"
                    , requested_roi->width, requested_roi->height, requested_roi->x, requested_roi->y
                    , stream_idx
                    , full.x(), full.y()
                );
            }
            else if (*granted == roi_t{ 0, 0, full.x(), full.y() })
            {
                // 전체 프레임은 `roi` 가 비어 있는 것으로 표현한다. 전체 범위를 받은 소스는 그 값을 그대로
                // 답하므로 여기서 비운다.
                granted.reset();
            }
            else
            {
                apply_roi(calib, *granted); // 캘리브레이션은 전달되는 이미지를 설명한다
            }
        }

        // 카메라에 쓰는 일은 스트림을 멈췄다 다시 시작하므로 락 밖에서 했고, 네 값은 한 락 아래에서 함께
        // 발행한다.
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_geometries.size()) { return; }
        stream_geometry_t& geometry = _stream_geometries[stream_idx];
        geometry.calib = std::move(calib);
        geometry.full_frame_resolution = full;
        geometry.roi = granted;
        geometry.frame_resolution = geometry.calib.frame_resolution; // `apply_roi` 가 이미 줄였다
    }

    std::size_t sensor_frame_provider::impl::stream_count() const
    {
        std::scoped_lock lk{ _geometry_mtx };
        return _stream_geometries.size();
    }

    calibration_t sensor_frame_provider::impl::get_calibration(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_geometries.size()) { return calibration_t{}; }
        return _stream_geometries[stream_idx].calib;
    }

    frame_format_t sensor_frame_provider::impl::get_frame_format(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_infos.size()) { return frame_format_t{}; }
        return _stream_infos[stream_idx].frame_format;
    }

    stream_descriptor_t sensor_frame_provider::impl::get_stream_descriptor(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_infos.size()) { return stream_descriptor_t{}; }
        return _stream_infos[stream_idx].descriptor;
    }

    Eigen::Vector2i sensor_frame_provider::impl::get_frame_resolution(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_geometries.size()) { return Eigen::Vector2i::Zero(); }
        return _stream_geometries[stream_idx].frame_resolution;
    }

    Eigen::Vector2i sensor_frame_provider::impl::get_full_frame_resolution(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_geometries.size()) { return Eigen::Vector2i::Zero(); }
        return _stream_geometries[stream_idx].full_frame_resolution;
    }

    std::optional<roi_t> sensor_frame_provider::impl::get_effective_roi(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _geometry_mtx };
        if (stream_idx >= _stream_geometries.size()) { return std::nullopt; }
        return _stream_geometries[stream_idx].roi;
    }

    float sensor_frame_provider::impl::get_current_update_rate(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _stream_stats_mtx };
        if (stream_idx >= _stream_stats.size()) { return 0.0f; }
        return _stream_stats[stream_idx].update_rate_fps;
    }

    uint64_t sensor_frame_provider::impl::get_frames_delivered(const std::size_t stream_idx) const
    {
        std::scoped_lock lk{ _stream_stats_mtx };
        if (stream_idx >= _stream_stats.size()) { return 0; }
        return _stream_stats[stream_idx].frames_delivered;
    }

    sync_stats_t sensor_frame_provider::impl::get_sync_stats() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _synchronizer ? _synchronizer->get_sync_stats() : sync_stats_t{};
    }

    void sensor_frame_provider::impl::set_roi(const std::size_t stream_idx, const std::optional<roi_t>& roi)
    {
        {
            std::scoped_lock lk{ _wake_cv_mtx };
            if (stream_idx >= _pending_roi_reqs.size()) { return; }
            _pending_roi_reqs[stream_idx] = roi_request_t{ .window = roi };
        }
        _wake_cv.notify_all();
    }

    bool sensor_frame_provider::impl::_has_pending_roi_request() const
    {
        for (const std::optional<roi_request_t>& request : _pending_roi_reqs)
        {
            if (request.has_value()) { return true; }
        }
        return false;
    }

    std::vector<std::size_t> sensor_frame_provider::impl::_apply_pending_rois()
    {
        std::vector<std::optional<roi_request_t>> requests;
        {
            std::scoped_lock lk{ _wake_cv_mtx };
            if (!this->_has_pending_roi_request()) { return {}; }
            requests = std::exchange(_pending_roi_reqs,
                std::vector<std::optional<roi_request_t>>(_pending_roi_reqs.size(), std::nullopt));
        }

        std::vector<std::size_t> reframed_streams;
        if (!_synchronizer) { return reframed_streams; }

        for (std::size_t stream_idx = 0; stream_idx < requests.size(); ++stream_idx)
        {
            if (!requests[stream_idx].has_value()) { continue; }

            // 전체 프레임으로 되돌리는 요청은 소스에 전체 범위를 쓰는 것이다.
            const Eigen::Vector2i full = this->get_full_frame_resolution(stream_idx);
            const roi_t want = requests[stream_idx]->window.value_or(roi_t{ 0, 0, full.x(), full.y() });

            const std::optional<roi_t> old_roi = this->get_effective_roi(stream_idx);
            this->_install_frame_geometry(*_synchronizer, stream_idx, want);
            const std::optional<roi_t> new_roi = this->get_effective_roi(stream_idx);

            // 픽셀 프레임이 그대로인 요청(거부됐거나 이미 걸린 값에 스냅)은 하류에 알릴 것이 없다.
            if (new_roi == old_roi) { continue; }

            spdlog::info("provider: stream {} is now framed {}", stream_idx, describe_roi(new_roi));
            reframed_streams.push_back(stream_idx);
        }

        return reframed_streams;
    }

    void sensor_frame_provider::impl::close()
    {
        _stop_thread();

        bool had_source = false;
        {
            std::scoped_lock lk{ _source_mtx };
            had_source = static_cast<bool>(_synchronizer);
            _synchronizer.reset(); // 스트림들을 닫는다
            _player.reset();
        }

        // 설명할 스트림이 없다.
        {
            std::scoped_lock lk{ _geometry_mtx };
            _stream_geometries.clear();
            _stream_infos.clear();
        }
        {
            std::scoped_lock lk{ _stream_stats_mtx };
            _stream_stats.clear();
        }

        if (had_source)
        {
            _notify_sensor_stream_end(stream_end_reason_t::completed);
        }
    }

    void sensor_frame_provider::impl::_start_thread()
    {
        _running.store(true);
        _thread = std::thread{ &sensor_frame_provider::impl::_polling_thread_proc, this };
    }

    void sensor_frame_provider::impl::_stop_thread()
    {
        { std::scoped_lock lk{ _wake_cv_mtx }; _running.store(false); }
        _wake_cv.notify_all(); // 어느 sleep 에 있든 깨운다
        if (_thread.joinable())
        {
            _thread.join();
        }
    }

    void sensor_frame_provider::impl::_polling_thread_proc()
    try
    {
        // 기다리는 seek 이 있으면 수행한다. 위치가 움직였으면 true.
        const auto apply_pending_seek = [this] {
            std::optional<seek_request_t> request;
            {
                std::scoped_lock lk{ _wake_cv_mtx };
                request = std::exchange(_pending_seek_req, std::nullopt);
            }
            if (!request.has_value()) { return false; }
            if (!_player) { return false; } // 라이브 카메라는 seek 할 곳이 없다

            switch (request->kind) {
            case seek_request_t::kind_t::begin:    _player->seek_begin(); break;
            case seek_request_t::kind_t::end:      _player->seek_end(); break;
            case seek_request_t::kind_t::timeline: _player->seek_timestamp(request->at); break;
            }

            // 스트림들이 옮겨졌으므로 synchronizer 가 들고 있던 것은 옛 위치다. 다음 fetch 가 새 위치에서
            // 싱크를 잡는다.
            if (_synchronizer) { _synchronizer->flush(); }
            return true;
        };

        // `until` 까지 잔다. 정지, seek, ROI, 그리고 일시정지 상태가 `paused_now` 에서 바뀌면 일찍 깬다.
        const auto sleep_until = [this](
            const std::chrono::steady_clock::time_point until,
            const bool paused_now)
        {
            std::unique_lock lk{ _wake_cv_mtx };
            _wake_cv.wait_until(lk, until, [&] {
                return !_running.load() || _pending_seek_req.has_value()
                    || this->_has_pending_roi_request() || _paused.load() != paused_now;
            });
        };

        // 재생 페이싱 앵커 (녹화 재생만)
        bool anchor_set = false;
        std::chrono::steady_clock::time_point anchor_wall{};
        timestamp_t anchor_ts{};

        // frameset fps EMA 의 벽시계 기준.
        bool have_last_wall = false;
        std::chrono::steady_clock::time_point last_wall{};

        // 스트림별 fps 는 그 스트림이 마지막으로 기여한 때부터 잰다.
        std::vector<std::chrono::steady_clock::time_point> stream_last_wall;
        std::vector<char> stream_has_last_wall;

        // frameset 하나를 스트림별 전달 카운터에 반영한다.
        const auto record_stream_delivery = [&](
            const synced_frameset& frameset,
            const std::chrono::steady_clock::time_point now)
        {
            const std::size_t count = frameset.stream_count();
            if (stream_last_wall.size() < count)
            {
                stream_last_wall.resize(count);
                stream_has_last_wall.resize(count, 0);
            }

            std::scoped_lock lk{ _stream_stats_mtx };
            if (_stream_stats.size() < count) { _stream_stats.resize(count); }

            for (std::size_t stream_idx = 0; stream_idx < count; ++stream_idx)
            {
                if (!frameset.stream_frameset(stream_idx)) { continue; }

                stream_delivery_stats_t& stats = _stream_stats[stream_idx];
                ++stats.frames_delivered;

                if (stream_has_last_wall[stream_idx])
                {
                    const double dt_sec = std::chrono::duration<double>{
                        now - stream_last_wall[stream_idx] }.count();
                    if (dt_sec > 1e-9)
                    {
                        const float inst = static_cast<float>(1.0 / dt_sec);
                        stats.update_rate_fps = (stats.update_rate_fps <= 0.0f)
                            ? inst
                            : (kFpsEmaAlpha * inst + (1.0f - kFpsEmaAlpha) * stats.update_rate_fps);
                    }
                }
                stream_last_wall[stream_idx] = now;
                stream_has_last_wall[stream_idx] = 1;
            }
        };

        // 녹화가 끝에서 처음으로 되돌려졌고 아직 frameset 을 내지 않았다.
        bool restarted = false;

        // 직전 fetch 가 frameset 없이 끝났는지.
        bool device_idle = false;

        // frameset 이 없으면 잠깐 물러난다. 실패를 반복하는 장치가 루프를 공회전시키지 않기 위해서다.
        // 경고는 조건이 시작될 때 한 번만 찍고, 회복은 루프 본문이 알린다.
        const auto idle_on_no_frame = [&](const std::string_view reason)
        {
            if (!std::exchange(device_idle, true)) {
                spdlog::warn("provider: {}, retrying", reason);
            }
            sleep_until(std::chrono::steady_clock::now() + kIdlePollSleep, _paused.load());
        };

        while (_running.load())
        {
            // 이 스레드만 소스를 움직이므로 아래의 fetch 는 여기서 seek 한 위치를 돌려준다. 점프는 그
            // frameset 에 앞서 알린다.
            const bool seeked = apply_pending_seek();
            if (seeked)
            {
                _need_repace.store(true);
                this->_notify_sensor_stream_reset();
            }

            // ROI 도 같다: 새 기하의 첫 frameset 에 앞서 알린다.
            const std::vector<std::size_t> reframed_streams = this->_apply_pending_rois();
            if (!reframed_streams.empty())
            {
                // 픽셀 프레임이 움직이면 프레임 사이에 들고 있던 것이 무효가 되고, 프레임 싱크는 모든
                // 스트림을 보고 하므로 reset 은 소스 전체에 한 번 나간다. 스트림별 통지는 어느 스트림의
                // 픽셀 좌표가 바뀌었는지를 보탠다.
                this->_notify_sensor_stream_reset();
                for (const std::size_t stream_idx : reframed_streams)
                {
                    this->_notify_sensor_frame_geometry_changed(stream_idx);
                }
            }

            // 멈춰 있고 frameset 을 요구하는 것도 없으면 쉰다. seek 과 ROI 는 도착한 위치와 새 기하를 보여
            // 줄 frameset 하나를 요구하므로 그때는 한 바퀴 더 돈다.
            if (_paused.load() && !seeked && reframed_streams.empty())
            {
                _need_repace.store(true);
                sleep_until(std::chrono::steady_clock::now() + kIdlePollSleep, true);
                continue;
            }

            std::optional<synced_frameset> frameset = _synchronizer->fetch_next_synced_frameset();

            if (!_running.load()) { break; }

            if (!frameset.has_value())
            {
                if (!_player)
                {
                    idle_on_no_frame("no frameset from device");
                    continue;
                }

                // 처음으로 되돌리는 것도 seek 이라 다음 바퀴가 수행한다. 되돌린 뒤에도 아무것도 나오지
                // 않으면 녹화가 비어 있는 것이라 끝낸다.
                if (_auto_repeat.load() && !restarted)
                {
                    restarted = true;
                    spdlog::debug("provider: recording reached its end, starting over");
                    this->_post_seek_request({ .kind = seek_request_t::kind_t::begin });
                    continue;
                }

                spdlog::info("provider: end of recording stream");
                this->_notify_sensor_stream_end(stream_end_reason_t::completed);
                this->pause(); // close / seek / play 까지 쉰다
                continue;
            }

            // 슬롯이 전부 빈 frameset 은 전달할 것이 없다.
            if (frameset->present_stream_count() == 0)
            {
                idle_on_no_frame("capture carried no image");
                continue;
            }

            if (std::exchange(device_idle, false)) {
                spdlog::info("provider: framesets from the device resumed");
            }

            restarted = false; // 전달 중이므로 다음 끝은 되돌릴 끝이다
            _frameset_seq.fetch_add(1);

            // EMA fps (벽시계 기준)
            const auto now = std::chrono::steady_clock::now();
            if (have_last_wall)
            {
                const double dt_sec = std::chrono::duration<double>{ now - last_wall }.count();
                if (dt_sec > 1e-9)
                {
                    const float inst = static_cast<float>(1.0 / dt_sec);
                    const float prev = _frameset_rate.load();
                    _frameset_rate.store(prev <= 0.0f ? inst : (kFpsEmaAlpha * inst + (1.0f - kFpsEmaAlpha) * prev));
                }
            }
            last_wall = now;
            have_last_wall = true;

            record_stream_delivery(*frameset, now);

            this->_notify_synced_frameset_update(*frameset);

            // frameset 이 실어 온 타임스탬프로 재생을 실시간 × 속도에 맞춘다.
            if (_player)
            {
                const float speed = (_speed.load() > 0.0f) ? _speed.load() : 1.0f;

                if (!anchor_set || _need_repace.exchange(false))
                {
                    anchor_set = true;
                    anchor_wall = std::chrono::steady_clock::now();
                    anchor_ts = frameset->timestamp();
                }
                else
                {
                    const double rel_ns = static_cast<double>((frameset->timestamp() - anchor_ts).count()) / speed;
                    const auto target = anchor_wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double, std::nano>{ rel_ns });
                    sleep_until(target, false);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        this->_end_stream_on_error(e.what());
    }
    catch (...)
    {
        this->_end_stream_on_error("unknown exception");
    }

    void sensor_frame_provider::impl::_end_stream_on_error(const std::string_view reason)
    {
        spdlog::error("provider: the polling thread stopped on {}", reason);
        _running.store(false);

        // 관찰자가 던져도 스레드는 여기서 조용히 끝난다.
        try { this->_notify_sensor_stream_end(stream_end_reason_t::failed); } catch (...) {}
    }

    std::vector<std::shared_ptr<synced_frameset_observer>> sensor_frame_provider::impl::_snapshot_observers() const
    {
        std::scoped_lock lk{ _observers_mtx };
        return _observers; // 사본. 콜백은 락 밖에서 돈다
    }

    void sensor_frame_provider::impl::_notify_synced_frameset_update(const synced_frameset& frameset)
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_synced_frameset_update(frameset); }
    }

    void sensor_frame_provider::impl::_notify_sensor_stream_reset()
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_stream_reset(); }
    }

    void sensor_frame_provider::impl::_notify_sensor_frame_geometry_changed(const std::size_t stream_idx)
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_frame_geometry_changed(stream_idx); }
    }

    void sensor_frame_provider::impl::_notify_sensor_stream_end(const stream_end_reason_t reason)
    {
        for (const auto& obs : _snapshot_observers()) { obs->on_sensor_stream_end(reason); }
    }

    void sensor_frame_provider::impl::play()
    {
        _need_repace.store(true);
        { std::scoped_lock lk{ _wake_cv_mtx }; _paused.store(false); }
        _wake_cv.notify_all();
    }

    void sensor_frame_provider::impl::pause()
    {
        { std::scoped_lock lk{ _wake_cv_mtx }; _paused.store(true); }
        _wake_cv.notify_all();
    }

    void sensor_frame_provider::impl::seek_recording_to_begin()
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::begin });
    }

    void sensor_frame_provider::impl::seek_recording_to_end()
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::end });
    }

    void sensor_frame_provider::impl::seek_recording_timeline(const timestamp_t timestamp)
    {
        this->_post_seek_request({ .kind = seek_request_t::kind_t::timeline, .at = timestamp });
    }

    void sensor_frame_provider::impl::_post_seek_request(const seek_request_t& request)
    {
        {
            // 가장 새것만 남긴다. 타임라인 드래그는 움직일 때마다 하나씩 올리는데 읽을 가치가 있는 것은
            // 마지막 위치뿐이다.
            std::scoped_lock lk{ _wake_cv_mtx };
            _pending_seek_req = request;
        }
        _wake_cv.notify_all();
    }

    std::chrono::nanoseconds sensor_frame_provider::impl::get_recording_length() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_recording_length() : std::chrono::nanoseconds{ 0 };
    }

    timestamp_t sensor_frame_provider::impl::get_first_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_first_record_timestamp() : timestamp_t{};
    }

    timestamp_t sensor_frame_provider::impl::get_last_record_timestamp() const
    {
        std::scoped_lock lk{ _source_mtx };
        return _player ? _player->get_last_record_timestamp() : timestamp_t{};
    }

    void sensor_frame_provider::impl::set_update_speed(float factor)
    {
        _speed.store(factor > 0.0f ? factor : 1.0f);
        _need_repace.store(true);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // sensor_frame_provider
    ///////////////////////////////////////////////////////////////////////////////////////////////

    sensor_frame_provider::sensor_frame_provider()
        : _imp{ std::make_unique<impl>() }
    { }

    sensor_frame_provider::~sensor_frame_provider() = default;

    void sensor_frame_provider::add_observer(std::shared_ptr<synced_frameset_observer> observer) { _imp->add_observer(std::move(observer)); }
    void sensor_frame_provider::remove_observer(const std::shared_ptr<synced_frameset_observer>& observer) { _imp->remove_observer(observer); }

    bool sensor_frame_provider::is_opened() const { return _imp->is_opened(); }
    bool sensor_frame_provider::open(const source_config_t& config) noexcept { return _imp->open(config); }
    bool sensor_frame_provider::open_synced(
        const std::span<const source_config_t> member_configs,
        const sync_options_t& sync_options) noexcept
    {
        return _imp->open_synced(member_configs, sync_options);
    }
    void sensor_frame_provider::close() { _imp->close(); }

    const std::string& sensor_frame_provider::get_source_name() const { return _imp->_source_name; }
    std::size_t sensor_frame_provider::stream_count() const { return _imp->stream_count(); }

    calibration_t sensor_frame_provider::get_calibration(const std::size_t stream_idx) const { return _imp->get_calibration(stream_idx); }
    frame_format_t sensor_frame_provider::get_frame_format(const std::size_t stream_idx) const { return _imp->get_frame_format(stream_idx); }
    stream_descriptor_t sensor_frame_provider::get_stream_descriptor(const std::size_t stream_idx) const { return _imp->get_stream_descriptor(stream_idx); }
    Eigen::Vector2i sensor_frame_provider::get_frame_resolution(const std::size_t stream_idx) const { return _imp->get_frame_resolution(stream_idx); }
    Eigen::Vector2i sensor_frame_provider::get_full_frame_resolution(const std::size_t stream_idx) const { return _imp->get_full_frame_resolution(stream_idx); }
    std::optional<roi_t> sensor_frame_provider::get_effective_roi(const std::size_t stream_idx) const { return _imp->get_effective_roi(stream_idx); }
    void sensor_frame_provider::set_roi(const std::size_t stream_idx, const std::optional<roi_t>& roi) { _imp->set_roi(stream_idx, roi); }

    float sensor_frame_provider::get_current_update_rate(const std::size_t stream_idx) const { return _imp->get_current_update_rate(stream_idx); }
    uint64_t sensor_frame_provider::get_frames_delivered(const std::size_t stream_idx) const { return _imp->get_frames_delivered(stream_idx); }
    float sensor_frame_provider::get_current_frameset_rate() const { return _imp->_frameset_rate.load(); }
    uint32_t sensor_frame_provider::get_current_frameset_seq() const { return _imp->_frameset_seq.load(); }
    sync_stats_t sensor_frame_provider::get_sync_stats() const { return _imp->get_sync_stats(); }

    bool sensor_frame_provider::is_paused() const { return _imp->_paused.load(); }
    void sensor_frame_provider::play() { _imp->play(); }
    void sensor_frame_provider::pause() { _imp->pause(); }

    void sensor_frame_provider::seek_recording_to_begin() { _imp->seek_recording_to_begin(); }
    void sensor_frame_provider::seek_recording_to_end() { _imp->seek_recording_to_end(); }
    void sensor_frame_provider::seek_recording_timeline(const timestamp_t timestamp) { _imp->seek_recording_timeline(timestamp); }

    std::chrono::nanoseconds sensor_frame_provider::get_recording_length() const { return _imp->get_recording_length(); }
    timestamp_t sensor_frame_provider::get_first_record_timestamp() const { return _imp->get_first_record_timestamp(); }
    timestamp_t sensor_frame_provider::get_last_record_timestamp() const { return _imp->get_last_record_timestamp(); }

    float sensor_frame_provider::get_update_speed() const { return _imp->_speed.load(); }
    void sensor_frame_provider::set_update_speed(const float factor) { _imp->set_update_speed(factor); }

    bool sensor_frame_provider::is_auto_repeat_enabled() const { return _imp->_auto_repeat.load(); }
    void sensor_frame_provider::set_auto_repeat(const bool enable) { _imp->_auto_repeat.store(enable); }

} // namespace hw

#pragma once
#include "calibration.hh"
#include "frameset_synchronizer.hh"
#include "frameset_observer.hh"
#include "sensor_frame_source.hh"
#include "source_config.hh"

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace hw
{
    // 소스 하나를 열어 그 스트림들을 소유하고, 폴링 스레드가 `frameset_synchronizer` 에서 `synced_frameset`
    // 을 당겨 관찰자들에게 밀어준다. 어떤 백엔드를 만드는지는 .cc 에만 있다.
    //
    // 세션에 속한 것(재생 위치, 일시정지, 속도)은 하나이고, 스트림 하나에 속한 것(캘리브레이션, ROI,
    // 전달 수)은 스트림 인덱스로 묻는다. 관찰자 콜백은 전부 폴링 스레드에서 나간다.
    class sensor_frame_provider final {
    public:
        sensor_frame_provider();
        ~sensor_frame_provider();

        sensor_frame_provider(const sensor_frame_provider&) = delete;
        sensor_frame_provider& operator=(const sensor_frame_provider&) = delete;
        sensor_frame_provider(sensor_frame_provider&&) = delete;
        sensor_frame_provider& operator=(sensor_frame_provider&&) = delete;

        void add_observer(std::shared_ptr<synced_frameset_observer> observer);
        void remove_observer(const std::shared_ptr<synced_frameset_observer>& observer);

        bool is_opened() const;

        // 소스 하나를 연다: 카메라 한 대, 또는 파일이 담은 스트림 수만큼의 스트림을 가진 녹화.
        // 스트림 수는 `stream_count()` 로 되읽는다.
        [[nodiscard]] bool open(const source_config_t& config) noexcept;

        // 라이브 카메라 여럿을 한 시각 축으로 묶어 연다. 스트림 인덱스는 `member_configs` 의 순서이고,
        // 각 멤버의 `roi` 는 그 스트림에 걸린다. 녹화 멤버와 2개 미만은 거절한다.
        // K4A 유선 싱크의 subordinate 는 master 보다 먼저 열린다. 열기가 곧 시작이기 때문이다.
        // 하나라도 실패하면 이미 연 것들을 닫고 실패한다.
        [[nodiscard]] bool open_synced(
            std::span<const source_config_t> member_configs,
            const sync_options_t& sync_options
        ) noexcept;

        void close();

        source_backend_t get_source_backend() const;
        const std::string& get_source_name() const;

        // 열린 소스가 내는 스트림 수. 아무것도 안 열려 있으면 0.
        std::size_t stream_count() const;

        // 관찰자가 `stream_idx` 에서 받는 이미지 기준이다. ROI 가 걸려 있으면 주점과 해상도가 그 윈도우에
        // 맞춰져 있다.
        calibration_t get_calibration(std::size_t stream_idx) const;

        frame_format_t get_frame_format(std::size_t stream_idx) const;
        stream_descriptor_t get_stream_descriptor(std::size_t stream_idx) const;

        Eigen::Vector2i get_frame_resolution(std::size_t stream_idx) const;
        Eigen::Vector2i get_full_frame_resolution(std::size_t stream_idx) const;

        std::optional<roi_t> get_effective_roi(std::size_t stream_idx) const; // nullopt: 전체 프레임

        // `stream_idx` 의 전달 이미지를 이 윈도우로 좁힌다. 비어 있으면 전체 프레임.
        // frameset 사이에 적용되고 카메라가 자기 증분에 스냅할 수 있으므로, 실제 값은 `get_effective_roi()` 로
        // 본다. 요청은 스트림마다 따로 들고 있다.
        void set_roi(std::size_t stream_idx, const std::optional<roi_t>& roi);

        // `stream_idx` 가 캡처를 기여한 frameset 의 초당 수. EMA 로 평활.
        float get_current_update_rate(std::size_t stream_idx) const;

        // 소스를 연 뒤 `stream_idx` 가 기여한 캡처 수.
        uint64_t get_frames_delivered(std::size_t stream_idx) const;

        float get_current_frameset_rate() const; // EMA 로 평활

        // 가장 새로운 frameset 의 순번. 열 때마다 0 부터 다시 센다. 프레임을 가리키는 식별자는
        // `sensor_frame::id()` 다.
        uint32_t get_current_frameset_seq() const;

        // 스트림들의 프레임 싱크 상태. 아무것도 안 열려 있으면 비어 있다.
        sync_stats_t get_sync_stats() const;

        bool is_paused() const;
        void play();
        void pause();

        // 녹화 소스 전용. 그 외에는 아무 일도 하지 않는다. 폴링 스레드가 frameset 사이에서 수행하며,
        // 일시정지 중이어도 도착한 위치의 frameset 을 하나 내준다. 여러 번 올리면 가장 새것만 남는다.
        void seek_recording_to_begin();
        void seek_recording_to_end();
        void seek_recording_timeline(timestamp_t timestamp);

        // 녹화 소스 전용. 그 외에는 0.
        std::chrono::nanoseconds get_recording_length() const;
        timestamp_t get_first_record_timestamp() const;
        timestamp_t get_last_record_timestamp() const;

        // 녹화 재생 속도 배율. 라이브 소스에는 걸리지 않는다.
        float get_update_speed() const;
        void  set_update_speed(float factor);

        // 녹화가 끝에 닿으면 처음부터 다시 돈다. 일시정지, 속도와 같은 재생 정책이다.
        bool is_auto_repeat_enabled() const;
        void set_auto_repeat(bool enable);

    private:
        struct impl;
        std::unique_ptr<impl> _imp;
    }; // class

} // namespace hw

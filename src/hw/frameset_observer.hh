#pragma once
#include "synced_frameset.hh"

#include <cstddef>

namespace hw
{
    enum class stream_end_reason_t
    {
        completed, // 소스의 프레임이 다 떨어졌거나, 닫힌 경우
        failed,    // 소스가 이어갈 수 없는 문제로 멈춘 경우
    };

    // provider 에 등록해 frameset 과 스트림 사건을 받는다.
    class synced_frameset_observer
    {
    public:
        synced_frameset_observer() = default;
        synced_frameset_observer(synced_frameset_observer&) = delete;
        synced_frameset_observer(synced_frameset_observer&&) = delete;
        synced_frameset_observer& operator=(synced_frameset_observer&) = delete;
        synced_frameset_observer& operator=(synced_frameset_observer&&) = delete;
        virtual ~synced_frameset_observer() = default;

        // 모든 스트림이 같은 순간의 캡처를 갖췄을 때. 슬롯이 빈 frameset 은 오지 않는다.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_synced_frameset_update(const synced_frameset& new_frameset) = 0;

        // 스트림 위치가 점프했을 때(소스 설치, seek, 되감기, ROI 변경). 프레임 사이에 들고 있던 상태를 버린다.
        // 한 스트림만 움직였어도 프레임 싱크가 전체를 다시 시작하므로 소스 전체에 한 번 알린다.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_sensor_stream_reset() = 0;

        // `stream_idx` 의 픽셀 좌표가 뜻을 잃었을 때(ROI 변경). 그 스트림의 픽셀 좌표에 매달린 상태를 버린다.
        // `on_sensor_stream_reset()` 뒤에 온다.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_sensor_frame_geometry_changed(std::size_t /*stream_idx*/) {}

        // 소스가 더 내놓을 것이 없을 때 한 번.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_sensor_stream_end(stream_end_reason_t reason) = 0;
    };

    // 스트림 하나만 읽는 관찰자.
    class single_stream_frameset_observer : public synced_frameset_observer
    {
    public:
        explicit single_stream_frameset_observer(const std::size_t read_stream_idx)
            : _read_stream_idx{ read_stream_idx }
        { }

        std::size_t read_stream_idx() const noexcept { return _read_stream_idx; }

        void on_synced_frameset_update(const synced_frameset& new_frameset) final
        {
            const sensor_frameset* frameset = new_frameset.stream_frameset(_read_stream_idx);
            if (!frameset) { return; }
            this->on_sensor_frameset_update(*frameset);
        }

        void on_sensor_frame_geometry_changed(const std::size_t stream_idx) final
        {
            if (stream_idx != _read_stream_idx) { return; }
            this->on_sensor_frame_geometry_changed();
        }

    protected:
        // 읽는 스트림의 이 순간의 캡처.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_sensor_frameset_update(const sensor_frameset& new_frameset) = 0;

        // 읽는 스트림의 픽셀 좌표가 뜻을 잃었을 때.
        // NOTE: 워커 스레드에서 불릴 수 있다.
        virtual void on_sensor_frame_geometry_changed() {}

    private:
        std::size_t _read_stream_idx;
    };

} // namespace hw

#pragma once
#include <opencv2/core.hpp>

#include <chrono>

namespace hw
{
    // One capture from a backend (wraps e.g. k4a::capture / ob::FrameSet).
    class sensor_frameset {
    public:
        virtual ~sensor_frameset() = default;

        virtual std::chrono::microseconds get_color_timestamp() const = 0;

        virtual bool has_color_image() const = 0;

        // get_color_image() may return a zero-copy view valid only while this
        // frameset is alive; clone it to outlive the frameset. 
        // Color is always exposed as BGR (CV_8UC3).
        // (backends convert from their native format)
        virtual cv::Mat get_color_image() const = 0;
    };

} // namespace hw

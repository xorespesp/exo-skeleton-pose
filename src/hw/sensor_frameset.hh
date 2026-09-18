#pragma once
#include "sensor_frame.hh"
#include "timestamp.hh"

#include <memory>
#include <stdexcept>
#include <utility>

namespace hw
{
    // Single frameset capture from a backend.
    class sensor_frameset final
    {
    public:
        explicit sensor_frameset(std::shared_ptr<sensor_frame> frame)
            : _frame{ std::move(frame) }
        {
            if (!_frame) { throw std::invalid_argument{ "sensor_frameset: no frame" }; }
        }

        const std::shared_ptr<sensor_frame>& frame() const noexcept { return _frame; }

        timestamp_t timestamp() const noexcept { return _frame->timestamp(); }

    private:
        std::shared_ptr<sensor_frame> _frame;
    };

} // namespace hw

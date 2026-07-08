#pragma once
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>

#include <imgui.h>

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace gui
{
    // Thread-safe spdlog sink that buffers formatted records for on-screen display.
    // Hook it into logging by adding an instance to a logger's sink list; the paired
    // `log_console` widget renders the buffered records as a colored console.
    class log_console_sink final : public spdlog::sinks::base_sink<std::mutex>
    {
    public:
        struct record_t
        {
            spdlog::level::level_enum level;
            std::string text; // fully formatted line (pattern applied, newline stripped)
        };

        // `capacity` caps the ring buffer; the oldest record is dropped past it.
        explicit log_console_sink(std::size_t capacity = 5000);

        // Run `fn(const std::deque<record_t>&)` while holding the sink lock. The UI
        // thread renders through this so it never copies the buffer per frame.
        template <typename _Fn>
        void access(_Fn&& fn)
        {
            std::lock_guard lk{ this->mutex_ };
            fn(static_cast<const std::deque<record_t>&>(_records));
        }

        void clear();

    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override;
        void flush_() override {}

    private:
        std::deque<record_t> _records;
        std::size_t _capacity;
    };

    // imgui log window backed by a `log_console_sink`. Owns the sink (shared with the
    // logger, so it outlives worker threads) and renders it as a colored, scrollable,
    // filterable console modeled on imgui's example log window.
    class log_console
    {
    public:
        log_console();

        // Sink to register on a logger (e.g. spdlog::default_logger()->sinks()).
        const std::shared_ptr<log_console_sink>& sink() const { return _sink; }

        // Renders the console (toolbar + scrolling view) into the current window/child
        // region. The caller owns the surrounding panel/layout.
        void draw();

    private:
        std::shared_ptr<log_console_sink> _sink;
        ImGuiTextFilter _filter;
        bool _auto_scroll{ true };
    };

} // namespace gui

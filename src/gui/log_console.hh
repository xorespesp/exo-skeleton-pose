#pragma once
#include <spdlog/common.h>
#include <spdlog/sinks/base_sink.h>

#include <imgui.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gui
{
    // Number of spdlog severities the console knows (trace..critical; `off` is not a record level).
    inline constexpr std::size_t kNumLogLevels = spdlog::level::critical + 1;

    // Thread-safe spdlog sink that buffers formatted records for on-screen display.
    // Hook it into logging by adding an instance to a logger's sink list; the paired
    // `log_console` widget renders the buffered records as a colored console.
    //
    // Keep this sink's level at trace: severity filtering belongs to the widget's view, and a
    // record dropped here is gone for good, so a later change of filter could not bring it back.
    class log_console_sink final : public spdlog::sinks::base_sink<std::mutex>
    {
    public:
        struct record_t
        {
            spdlog::level::level_enum level;
            std::string text; // fully formatted line (pattern applied, newline stripped)
        };

        // What `access()` hands the reader while the sink lock is held.
        struct view_t
        {
            const std::deque<record_t>& records;
            const std::array<std::size_t, kNumLogLevels>& counts; // buffered records per level
            uint64_t version; // bumps on every record/clear; an unchanged value means `records` is unchanged
        };

        // `capacity` caps the ring buffer; the oldest record is dropped past it.
        explicit log_console_sink(std::size_t capacity = 20000);

        // Run `fn(const view_t&)` while holding the sink lock. The UI thread renders
        // through this so it never copies the buffer per frame.
        template <typename _Fn>
        void access(_Fn&& fn)
        {
            std::lock_guard lk{ this->mutex_ };
            fn(view_t{ _records, _counts, _version });
        }

        std::array<std::size_t, kNumLogLevels> counts();

        void clear();

    protected:
        void sink_it_(const spdlog::details::log_msg& msg) override;
        void flush_() override {}

    private:
        std::deque<record_t> _records;
        std::array<std::size_t, kNumLogLevels> _counts{}; // kept in sync with `_records` across drops
        uint64_t _version{ 0 };
        std::size_t _capacity;
    };

    // imgui log window backed by a `log_console_sink`. Owns the sink (shared with the
    // logger, so it outlives worker threads) and renders it as a colored, scrollable console,
    // filterable by severity (per-level toggles) and by substring.
    //
    // The filters are view-only: every record stays in the buffer, so turning a level back on
    // brings its past records back with it.
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
        bool _draw_toolbar();                                // returns true when Copy was pressed
        void _rebuild_visible(const log_console_sink::view_t& v); // reindex the records passing the filters

        bool _level_shown(spdlog::level::level_enum lvl) const {
            return (_level_mask & (1u << static_cast<uint32_t>(lvl))) != 0;
        }

    private:
        std::shared_ptr<log_console_sink> _sink;
        ImGuiTextFilter _filter;
        uint32_t _level_mask; // bit k = show severity k; trace hidden by default (it is the flood-prone one)
        bool _auto_scroll{ true };

        // Records passing the filters, as indices into the sink's buffer. Rebuilt only when the
        // buffer or a filter changed, so the list clipper still drives the (filtered) view
        // instead of walking every record every frame.
        std::vector<int> _visible;
        uint64_t _visible_version{ 0 }; // buffer version `_visible` was built from
        bool _visible_dirty{ true };
    };

} // namespace gui

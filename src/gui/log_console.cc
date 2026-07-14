#include "log_console.hh"

#include <format>

namespace gui
{
    namespace
    {
        // Colored-console palette: whole line is tinted by its severity.
        ImVec4 level_color(spdlog::level::level_enum lvl)
        {
            switch (lvl)
            {
            case spdlog::level::trace:    return { 0.60f, 0.60f, 0.60f, 1.0f }; // gray
            case spdlog::level::debug:    return { 0.40f, 0.80f, 1.00f, 1.0f }; // cyan
            case spdlog::level::info:     return { 0.55f, 0.85f, 0.45f, 1.0f }; // green
            case spdlog::level::warn:     return { 1.00f, 0.80f, 0.25f, 1.0f }; // yellow
            case spdlog::level::err:      return { 1.00f, 0.40f, 0.40f, 1.0f }; // red
            case spdlog::level::critical: return { 1.00f, 0.30f, 0.85f, 1.0f }; // magenta
            default:                      return ImGui::GetStyleColorVec4(ImGuiCol_Text);
            }
        }

        // Toolbar chip label per severity.
        const char* level_tag(spdlog::level::level_enum lvl)
        {
            switch (lvl)
            {
            case spdlog::level::trace:    return "TRC";
            case spdlog::level::debug:    return "DBG";
            case spdlog::level::info:     return "INF";
            case spdlog::level::warn:     return "WRN";
            case spdlog::level::err:      return "ERR";
            case spdlog::level::critical: return "CRT";
            default:                      return "???";
            }
        }

        // Everything but trace: the default view, since trace is the flood-prone level.
        constexpr uint32_t kDefaultLevelMask =
            ((1u << kNumLogLevels) - 1u) & ~(1u << static_cast<uint32_t>(spdlog::level::trace));
    } // namespace

    log_console_sink::log_console_sink(std::size_t capacity)
        : _capacity{ capacity == 0 ? 1 : capacity }
    {
        // Self-contained pattern (no color-range markers; the widget colors per level).
        this->set_pattern("[%H:%M:%S.%e] [%l] %v");
    }

    void log_console_sink::sink_it_(const spdlog::details::log_msg& msg)
    {
        // base_sink::log() already holds `mutex_` around this call.
        spdlog::memory_buf_t formatted;
        this->formatter_->format(msg, formatted);

        std::string text{ formatted.data(), formatted.size() };
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) { text.pop_back(); }

        _records.push_back({ msg.level, std::move(text) });
        ++_counts[static_cast<std::size_t>(msg.level)];

        if (_records.size() > _capacity)
        {
            --_counts[static_cast<std::size_t>(_records.front().level)];
            _records.pop_front();
        }
        ++_version;
    }

    std::array<std::size_t, kNumLogLevels> log_console_sink::counts()
    {
        std::lock_guard lk{ this->mutex_ };
        return _counts;
    }

    void log_console_sink::clear()
    {
        std::lock_guard lk{ this->mutex_ };
        _records.clear();
        _counts.fill(0);
        ++_version;
    }

    log_console::log_console()
        : _sink{ std::make_shared<log_console_sink>() }
        , _level_mask{ kDefaultLevelMask }
    {
    }

    // Toolbar: actions, then one checkbox per severity (labeled with how many records of it the
    // buffer holds), then the substring filter.
    bool log_console::_draw_toolbar()
    {
        if (ImGui::Button("Options")) { ImGui::OpenPopup("log_options"); }
        if (ImGui::BeginPopup("log_options"))
        {
            ImGui::Checkbox("Auto-scroll", &_auto_scroll);
            ImGui::EndPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) { _sink->clear(); }
        ImGui::SameLine();
        const bool do_copy = ImGui::Button("Copy");

        const auto counts = _sink->counts();
        for (std::size_t i = 0; i < kNumLogLevels; ++i)
        {
            const auto lvl = static_cast<spdlog::level::level_enum>(i);
            bool shown = this->_level_shown(lvl);

            ImGui::SameLine();
            const std::string label = std::format("{} {}###lvl{}", level_tag(lvl), counts[i], i);
            if (ImGui::Checkbox(label.c_str(), &shown))
            {
                _level_mask ^= (1u << static_cast<uint32_t>(lvl));
                _visible_dirty = true;
            }
            ImGui::SetItemTooltip("%s: %zu record(s) buffered.", spdlog::level::to_string_view(lvl).data(), counts[i]);
        }

        ImGui::SameLine();
        if (_filter.Draw("Filter", -80.0f)) { _visible_dirty = true; }

        return do_copy;
    }

    void log_console::_rebuild_visible(const log_console_sink::view_t& v)
    {
        _visible.clear();
        _visible.reserve(v.records.size());
        for (int i = 0; i < static_cast<int>(v.records.size()); ++i)
        {
            const auto& rec = v.records[i];
            if (!this->_level_shown(rec.level)) { continue; }
            if (!_filter.PassFilter(rec.text.c_str())) { continue; }
            _visible.push_back(i);
        }
    }

    void log_console::draw()
    {
        const bool do_copy = this->_draw_toolbar();

        ImGui::Separator();

        if (ImGui::BeginChild("scroll", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar))
        {
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

            _sink->access([&](const log_console_sink::view_t& v) {
                if (_visible_dirty || _visible_version != v.version)
                {
                    this->_rebuild_visible(v);
                    _visible_version = v.version;
                    _visible_dirty = false;
                }

                // Copy takes the whole filtered view, not just the rows the clipper drew.
                if (do_copy)
                {
                    std::string all;
                    for (const int idx : _visible) { all += v.records[idx].text; all += '\n'; }
                    ImGui::SetClipboardText(all.c_str());
                }

                ImGuiListClipper clipper;
                clipper.Begin(static_cast<int>(_visible.size()));
                while (clipper.Step())
                {
                    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
                    {
                        const auto& rec = v.records[_visible[i]];
                        ImGui::PushStyleColor(ImGuiCol_Text, level_color(rec.level));
                        ImGui::TextUnformatted(rec.text.c_str());
                        ImGui::PopStyleColor();
                    }
                }
                clipper.End();
            });

            ImGui::PopStyleVar();

            // Stick to the bottom while the user hasn't scrolled up.
            if (_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) { ImGui::SetScrollHereY(1.0f); }
        }
        ImGui::EndChild();
    }

} // namespace gui

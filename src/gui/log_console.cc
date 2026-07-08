#include "log_console.hh"

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
    } // namespace

    log_console_sink::log_console_sink(std::size_t capacity)
        : _capacity{ capacity }
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
        if (_records.size() > _capacity) { _records.pop_front(); }
    }

    void log_console_sink::clear()
    {
        std::lock_guard lk{ this->mutex_ };
        _records.clear();
    }

    log_console::log_console()
        : _sink{ std::make_shared<log_console_sink>() }
    {
    }

    void log_console::draw()
    {
        // ----- toolbar -----
        if (ImGui::Button("Options")) { ImGui::OpenPopup("log_options"); }
        if (ImGui::BeginPopup("log_options"))
        {
            ImGui::Checkbox("Auto-scroll", &_auto_scroll);
            ImGui::EndPopup();
        }
        ImGui::SameLine();
        const bool do_clear = ImGui::Button("Clear");
        ImGui::SameLine();
        const bool do_copy = ImGui::Button("Copy");
        ImGui::SameLine();
        _filter.Draw("Filter", -80.0f);

        ImGui::Separator();

        if (ImGui::BeginChild("scroll", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar))
        {
            if (do_clear) { _sink->clear(); }
            if (do_copy) { ImGui::LogToClipboard(); }

            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

            const auto draw_line = [](const log_console_sink::record_t& rec) {
                ImGui::PushStyleColor(ImGuiCol_Text, level_color(rec.level));
                ImGui::TextUnformatted(rec.text.c_str());
                ImGui::PopStyleColor();
            };

            _sink->access([&](const std::deque<log_console_sink::record_t>& records) {
                if (_filter.IsActive())
                {
                    // Filtered view: line count is unknown up front, so skip the clipper.
                    for (const auto& rec : records)
                    {
                        if (_filter.PassFilter(rec.text.c_str())) { draw_line(rec); }
                    }
                }
                else
                {
                    ImGuiListClipper clipper;
                    clipper.Begin(static_cast<int>(records.size()));
                    while (clipper.Step())
                    {
                        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) { draw_line(records[i]); }
                    }
                    clipper.End();
                }
            });

            if (do_copy) { ImGui::LogFinish(); }

            ImGui::PopStyleVar();

            // Stick to the bottom while the user hasn't scrolled up.
            if (_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) { ImGui::SetScrollHereY(1.0f); }
        }
        ImGui::EndChild();
    }

} // namespace gui

#pragma once
#include <CLI/CLI.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace app
{
    // Frame source cli options
    struct source_options
    {
        uint32_t device_index{ 0 };
        std::string input_path; // mkv recording (else live device)
        double tag_size_m{ 0.05 };
        std::optional<int32_t> exposure_us;
        std::optional<int32_t> gain;

        bool is_recording() const { return !input_path.empty(); }
    };

    void add_source_options(CLI::App& app, source_options& o);

} // namespace app

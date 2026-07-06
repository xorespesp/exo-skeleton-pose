#include "cli_options.hh"
#include "gui/debug_gui_app.hh"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <exception>

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_on(spdlog::level::info);

    CLI::App cli{ "exo-skeleton-pose" };
    app::source_options opt;
    app::add_source_options(cli, opt);
    CLI11_PARSE(cli, argc, argv);

    try
    {
        return gui::debug_gui_app{ opt }.run();
    }
    catch (const std::exception& e)
    {
        spdlog::error("fatal: {}", e.what());
        return -1;
    }
}

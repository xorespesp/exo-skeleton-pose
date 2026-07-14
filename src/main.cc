#include "cli_options.hh"
#include "gui/debugger_app.hh"
#include "net/exo_pose_server.hh"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <exception>

int main(int argc, char** argv)
{
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::flush_on(spdlog::level::info);

    // The logger passes every severity; each sink decides what it keeps. The terminal takes
    // info and up, while the debugger's in-GUI console attaches a sink that records everything
    // (its severity toggles filter the view, so a record must be there to be revealed later).
    spdlog::set_level(spdlog::level::trace);
    for (auto& sink : spdlog::default_logger()->sinks()) { sink->set_level(spdlog::level::info); }

    CLI::App cli{ "exo-skeleton-pose" };

    // Bare invocation launches the debugger GUI, which embeds a WebSocket server the operator
    // starts/stops from the Server menu.
    app::source_options gui_opt;
    uint16_t gui_port{ 9002 };
    app::add_source_options(cli, gui_opt);
    cli.add_option("-p,--port", gui_port, "WebSocket listen port for the embedded server")->default_val(9002);

    // `serve` launches a headless WebSocket pose server instead.
    app::source_options serve_opt;
    uint16_t serve_port{ 9002 };
    CLI::App* serve = cli.add_subcommand("serve", "Run the headless WebSocket pose server");
    app::add_source_options(*serve, serve_opt);
    serve->add_option("-p,--port", serve_port, "WebSocket listen port")->default_val(9002);

    CLI11_PARSE(cli, argc, argv);

    try
    {
        if (serve->parsed()) {
            return net::exo_pose_server{ serve_port, serve_opt }.run();
        }

        // Debugger GUI
        return gui::debugger_app{ gui_opt, gui_port }.run();
    }
    catch (const std::exception& e)
    {
        spdlog::error("fatal: {}", e.what());
        return -1;
    }
}

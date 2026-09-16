#pragma once
#include "app_base.hh"
#include "app_renderer_sdl3.hh"

#include <memory>

namespace gui
{
    // Displays camera and recording frames through the shared provider interface.
    class camera_test_app final : public app_base<app_renderer_sdl3>
    {
    public:
        camera_test_app();
        ~camera_test_app() override;

        int run();
        void render_ui() override;

    private:
        struct context_t;
        std::unique_ptr<context_t> _ctx;
    };
}

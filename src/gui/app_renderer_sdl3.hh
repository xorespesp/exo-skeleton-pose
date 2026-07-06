#pragma once
#include <SDL3/SDL.h>
#include <imgui.h>

#include <memory>
#include <string>

namespace gui
{
    // SDL3 + SDL_Renderer3 window that owns the ImGui/ImPlot/ImPlot3D lifecycle.
    class app_renderer_sdl3 final
    {
    public:
        app_renderer_sdl3() = default;
        ~app_renderer_sdl3();

        app_renderer_sdl3(const app_renderer_sdl3&) = delete;
        app_renderer_sdl3& operator=(const app_renderer_sdl3&) = delete;

        bool is_created() const { return _created; }
        bool create(const std::string& title, int width, int height);
        void destroy();

        bool poll();
        void new_frame();
        void render_frame();

        SDL_Renderer* sdl_renderer() const { return _renderer.get(); }
        ImVec2 window_size() const;
        float dpi_scale() const { return _dpi_scale; }

    private:
        void _setup_style();
        void _apply_dpi_scale(float scale); // re-scale style/fonts from the base

    private:
        bool _created{ false };
        std::shared_ptr<SDL_Window> _window;
        std::shared_ptr<SDL_Renderer> _renderer;
        ImVec4 _bg_color{ 0.15f, 0.16f, 0.21f, 1.00f };
        ImGuiStyle _base_style; // style at scale 1.0 (rescaled on DPI change)
        float _dpi_scale{ 1.0f };
    };

} // namespace gui

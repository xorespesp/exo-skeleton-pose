#include "app_renderer_sdl3.hh"

#include <spdlog/spdlog.h>

#include <implot.h>
#include <implot3d.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

namespace gui
{
    namespace
    {
        // Initializes SDL once for the whole process; SDL_Quit on shutdown.
        class sdl_environment final
        {
        public:
            static void initialize() { static sdl_environment inst{}; }

            sdl_environment(const sdl_environment&) = delete;
            sdl_environment& operator=(const sdl_environment&) = delete;
            ~sdl_environment() { ::SDL_Quit(); }

        private:
            sdl_environment()
            {
#if defined(SDL_MAIN_HANDLED)
                ::SDL_SetMainReady();
#endif
                if (!::SDL_Init(SDL_INIT_VIDEO))
                {
                    spdlog::error("SDL_Init failed: {}", ::SDL_GetError());
                    std::exit(-1);
                }
            }
        };
    } // namespace

    app_renderer_sdl3::~app_renderer_sdl3()
    {
        if (this->is_created()) { this->destroy(); }
    }

    bool app_renderer_sdl3::create(const std::string& title, int width, int height)
    {
        if (this->is_created()) { return false; }

        sdl_environment::initialize();

        const auto flags = static_cast<SDL_WindowFlags>(
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);

        _window.reset(::SDL_CreateWindow(title.c_str(), width, height, flags), ::SDL_DestroyWindow);
        if (!_window)
        {
            spdlog::error("Failed to create SDL window: {}", ::SDL_GetError());
            return false;
        }

        _renderer.reset(::SDL_CreateRenderer(_window.get(), nullptr), ::SDL_DestroyRenderer);
        if (!_renderer)
        {
            spdlog::error("Failed to create SDL renderer: {}", ::SDL_GetError());
            return false;
        }
        ::SDL_SetRenderVSync(_renderer.get(), 1);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();
        ImPlot3D::CreateContext();
        ::ImGui_ImplSDL3_InitForSDLRenderer(_window.get(), _renderer.get());
        ::ImGui_ImplSDLRenderer3_Init(_renderer.get());

        this->_setup_style();
        ImPlot::StyleColorsDark();
        _base_style = ImGui::GetStyle(); // snapshot at scale 1.0

        // Scale UI to the current display's DPI (updated live on scale changes).
        const float scale = ::SDL_GetWindowDisplayScale(_window.get());
        this->_apply_dpi_scale(scale > 0.0f ? scale : 1.0f);

        _created = true;
        return true;
    }

    void app_renderer_sdl3::destroy()
    {
        if (!this->is_created()) { return; }

        ::ImGui_ImplSDLRenderer3_Shutdown();
        ::ImGui_ImplSDL3_Shutdown();
        ImPlot3D::DestroyContext();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();

        _renderer.reset();
        _window.reset();
        _created = false;
    }

    bool app_renderer_sdl3::poll()
    {
        bool should_close = false;

        SDL_Event event;
        while (::SDL_PollEvent(&event))
        {
            ::ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) { should_close = true; }
            if (event.window.windowID != ::SDL_GetWindowID(_window.get())) { continue; }
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) { should_close = true; }
            if (event.type == SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED)
            {
                const float scale = ::SDL_GetWindowDisplayScale(_window.get());
                this->_apply_dpi_scale(scale > 0.0f ? scale : 1.0f);
            }
        }
        return !should_close;
    }

    void app_renderer_sdl3::_apply_dpi_scale(float scale)
    {
        _dpi_scale = scale;
        ImGuiStyle& style = ImGui::GetStyle();
        style = _base_style;          // reset to unscaled base, then scale sizes
        style.ScaleAllSizes(scale);
        ImGui::GetIO().FontGlobalScale = scale;
    }

    void app_renderer_sdl3::new_frame()
    {
        ::ImGui_ImplSDLRenderer3_NewFrame();
        ::ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
    }

    void app_renderer_sdl3::render_frame()
    {
        ImGui::Render();
        ::SDL_SetRenderDrawColor(_renderer.get(),
            static_cast<Uint8>(_bg_color.x * 255),
            static_cast<Uint8>(_bg_color.y * 255),
            static_cast<Uint8>(_bg_color.z * 255),
            static_cast<Uint8>(_bg_color.w * 255));
        ::SDL_RenderClear(_renderer.get());
        ::ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), _renderer.get());
        ::SDL_RenderPresent(_renderer.get());
    }

    ImVec2 app_renderer_sdl3::window_size() const
    {
        int w{}, h{};
        ::SDL_GetWindowSize(_window.get(), &w, &h);
        return ImVec2{ static_cast<float>(w), static_cast<float>(h) };
    }

    void app_renderer_sdl3::_setup_style()
    {
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 5.3f;
        style.FrameRounding = 2.3f;
        style.ScrollbarRounding = 0.0f;

        ImVec4* c = style.Colors;
        c[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.90f, 0.90f);
        c[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
        c[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.09f, 0.15f, 1.00f);
        c[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
        c[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.05f, 0.10f, 0.85f);
        c[ImGuiCol_Border] = ImVec4(0.70f, 0.70f, 0.70f, 0.65f);
        c[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
        c[ImGuiCol_FrameBg] = ImVec4(0.00f, 0.00f, 0.01f, 1.00f);
        c[ImGuiCol_FrameBgHovered] = ImVec4(0.90f, 0.80f, 0.80f, 0.40f);
        c[ImGuiCol_FrameBgActive] = ImVec4(0.90f, 0.65f, 0.65f, 0.45f);
        c[ImGuiCol_TitleBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.83f);
        c[ImGuiCol_TitleBgCollapsed] = ImVec4(0.40f, 0.40f, 0.80f, 0.20f);
        c[ImGuiCol_TitleBgActive] = ImVec4(0.00f, 0.00f, 0.00f, 0.87f);
        c[ImGuiCol_MenuBarBg] = ImVec4(0.01f, 0.01f, 0.02f, 0.80f);
        c[ImGuiCol_ScrollbarBg] = ImVec4(0.20f, 0.25f, 0.30f, 0.60f);
        c[ImGuiCol_ScrollbarGrab] = ImVec4(0.55f, 0.53f, 0.55f, 0.51f);
        c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.56f, 0.56f, 0.56f, 1.00f);
        c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.56f, 0.56f, 0.56f, 0.91f);
        c[ImGuiCol_CheckMark] = ImVec4(0.90f, 0.90f, 0.90f, 0.83f);
        c[ImGuiCol_SliderGrab] = ImVec4(0.70f, 0.70f, 0.70f, 0.62f);
        c[ImGuiCol_SliderGrabActive] = ImVec4(0.30f, 0.30f, 0.30f, 0.84f);
        c[ImGuiCol_Button] = ImVec4(0.48f, 0.72f, 0.89f, 0.49f);
        c[ImGuiCol_ButtonHovered] = ImVec4(0.50f, 0.69f, 0.99f, 0.68f);
        c[ImGuiCol_ButtonActive] = ImVec4(0.80f, 0.50f, 0.50f, 1.00f);
        c[ImGuiCol_Header] = ImVec4(0.18f, 0.22f, 0.34f, 1.00f);
        c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.33f, 0.50f, 1.00f);
        c[ImGuiCol_HeaderActive] = ImVec4(0.32f, 0.42f, 0.62f, 1.00f);
        c[ImGuiCol_Separator] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
        c[ImGuiCol_SeparatorHovered] = ImVec4(0.70f, 0.60f, 0.60f, 1.00f);
        c[ImGuiCol_SeparatorActive] = ImVec4(0.90f, 0.70f, 0.70f, 1.00f);
        c[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.85f);
        c[ImGuiCol_ResizeGripHovered] = ImVec4(1.00f, 1.00f, 1.00f, 0.60f);
        c[ImGuiCol_ResizeGripActive] = ImVec4(1.00f, 1.00f, 1.00f, 0.90f);
        c[ImGuiCol_PlotLines] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
        c[ImGuiCol_PlotLinesHovered] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
        c[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
        c[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
        c[ImGuiCol_TextSelectedBg] = ImVec4(0.00f, 0.00f, 1.00f, 0.35f);
        c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);
    }

} // namespace gui

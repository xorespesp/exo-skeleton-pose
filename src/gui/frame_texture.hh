#pragma once
#include <SDL3/SDL.h>
#include <imgui.h>
#include <opencv2/core.hpp>

namespace gui
{
    // Streams a cv::Mat (BGR/BGRA/gray) into an SDL texture for ImGui::Image.
    class frame_texture
    {
    public:
        explicit frame_texture(SDL_Renderer* renderer) : _renderer{ renderer } {}
        ~frame_texture();

        frame_texture(const frame_texture&) = delete;
        frame_texture& operator=(const frame_texture&) = delete;

        bool update(const cv::Mat& image);      // re/allocates on size change
        bool valid() const { return _tex != nullptr; }
        ImTextureID id() const { return reinterpret_cast<ImTextureID>(_tex); }
        int width() const { return _w; }
        int height() const { return _h; }

    private:
        SDL_Renderer* _renderer{ nullptr };
        SDL_Texture* _tex{ nullptr };
        int _w{ 0 }, _h{ 0 };
        cv::Mat _rgba; // scratch conversion buffer
    };

} // namespace gui

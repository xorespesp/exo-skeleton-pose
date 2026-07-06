#include "frame_texture.hh"

#include <opencv2/imgproc.hpp>

namespace gui
{
    frame_texture::~frame_texture()
    {
        if (_tex) { ::SDL_DestroyTexture(_tex); }
    }

    bool frame_texture::update(const cv::Mat& image)
    {
        if (image.empty() || !_renderer) { return false; }

        // SDL_PIXELFORMAT_RGBA32 has R,G,B,A byte order, matching cv ...2RGBA.
        switch (image.channels())
        {
        case 3: cv::cvtColor(image, _rgba, cv::COLOR_BGR2RGBA); break;
        case 4: cv::cvtColor(image, _rgba, cv::COLOR_BGRA2RGBA); break;
        default: cv::cvtColor(image, _rgba, cv::COLOR_GRAY2RGBA); break;
        }

        if (!_tex || _w != _rgba.cols || _h != _rgba.rows)
        {
            if (_tex) { ::SDL_DestroyTexture(_tex); _tex = nullptr; }
            _tex = ::SDL_CreateTexture(_renderer, SDL_PIXELFORMAT_RGBA32,
                SDL_TEXTUREACCESS_STREAMING, _rgba.cols, _rgba.rows);
            if (!_tex) { return false; }
            _w = _rgba.cols;
            _h = _rgba.rows;
        }

        return ::SDL_UpdateTexture(_tex, nullptr, _rgba.data, static_cast<int>(_rgba.step));
    }

} // namespace gui

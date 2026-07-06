#pragma once
#include <stdexcept>
#include <utility>

namespace gui
{
    // CRTP host that drives the per-frame loop of a concrete renderer.
    template <class _TyRenderer>
    class app_base
    {
    public:
        using renderer_type = _TyRenderer;

        app_base() = default;
        virtual ~app_base() = default;

        renderer_type& renderer() { return _renderer; }
        const renderer_type& renderer() const { return _renderer; }

        bool is_created() const { return _renderer.is_created(); }

        template <typename... _Args>
        bool create(_Args&&... args) { return _renderer.create(std::forward<_Args>(args)...); }

        void destroy() { _renderer.destroy(); }

        // Returns false once the window was closed.
        bool poll()
        {
            const bool alive = _renderer.poll();
            if (alive)
            {
                _renderer.new_frame();
                this->render_ui();
                _renderer.render_frame();
            }
            return alive;
        }

        void run() { while (this->poll()); }

        // Called once per frame; overridden by the concrete app.
        virtual void render_ui() { throw std::runtime_error{ "render_ui() not implemented" }; }

    private:
        renderer_type _renderer;
    };

} // namespace gui

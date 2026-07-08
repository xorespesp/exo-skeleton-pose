#pragma once
#include "dsp/one_euro_filter.hh"

#include <Eigen/Geometry>

#include <cmath>
#include <memory>

namespace pose
{
    // ---------------------------------------------------------------------------
    // Swappable rotation-smoothing kernel
    // ---------------------------------------------------------------------------
    //
    // A kernel smooths a stream of absolute rotations, one sample per fresh frame.
    // Occlusion hold / reseed policy lives in the estimator;
    // the kernel only owns the smoothing (and its own hemisphere continuity).
    enum class rotation_filter_kind
    {
        one_euro, // SLERP-based One Euro
        // TODO: Add savgol, slerp_ema, ... 
    };

    struct one_euro_params
    {
        double min_cutoff_hz = 1.0; // lower -> smoother at rest, more lag
        double beta = 0.05;         // higher -> more responsive in motion (0 -> SLERP-EMA)
        double dcutoff_hz = 1.0;    // cutoff for the speed-derivative low-pass
    };

    // Full kernel configuration bundle.
    struct rotation_filter_config
    {
        rotation_filter_kind kind = rotation_filter_kind::one_euro;
        one_euro_params one_euro{};
        // TODO: Add savgol options, slerp_ema options, ... 
    };

    struct rotation_filter_base
    {
        virtual ~rotation_filter_base() = default;

        // Live-apply parameters. a kernel reads only its own slice of the config.
        virtual void configure(const rotation_filter_config& cfg) = 0;

        // (Re)seed the kernel with a known-good rotation. (cold start / after a gap)
        virtual void reset(const Eigen::Quaterniond& q) = 0;

        // Smooth one sample. `dt_sec` is the elapsed time since the previous fresh sample (> 0).
        // Returns the smoothed absolute rotation.
        // NOTE: The caller supplies a hemisphere-continuous stream.
        //       (no double-cover sign flips between samples)
        [[nodiscard]] virtual Eigen::Quaterniond filter(const Eigen::Quaterniond& q, double dt_sec) = 0;
    };

    // One Euro on SO(3): the value low-pass is a SLERP toward the new sample with
    // an adaptive factor; the speed derivative uses a scalar dsp::LowPassFilter.
    class one_euro_rotation_filter final : public rotation_filter_base
    {
    public:
        explicit one_euro_rotation_filter(const one_euro_params& cfg) noexcept
            : _cfg{ cfg }
        { }

        void configure(const rotation_filter_config& cfg) override
        {
            _cfg = cfg.one_euro;
        }

        void reset(const Eigen::Quaterniond& q) override
        {
            _prev = q.normalized();
            _dspeed.reset();
            _init = true;
        }

        Eigen::Quaterniond filter(const Eigen::Quaterniond& q_in, double dt_sec) override
        {
            if (!_init) { 
                this->reset(q_in);
                return _prev;
            }

            const Eigen::Quaterniond q = q_in.normalized(); // NOTE: caller guarantees hemisphere continuity

            const double speed = _prev.angularDistance(q) / dt_sec; // rad/s
            const double sp = _dspeed.filter(speed, dsp::alpha(_cfg.dcutoff_hz, dt_sec));
            const double cutoff = _cfg.min_cutoff_hz + _cfg.beta * std::abs(sp);

            _prev = _prev.slerp(dsp::alpha(cutoff, dt_sec), q).normalized();
            return _prev;
        }

    private:
        one_euro_params _cfg;
        Eigen::Quaterniond _prev{ Eigen::Quaterniond::Identity() };
        dsp::LowPassFilter _dspeed;
        bool _init{ false };
    };

    [[nodiscard]] inline std::unique_ptr<rotation_filter_base>
    make_rotation_filter(const rotation_filter_config& cfg)
    {
        switch (cfg.kind) {
        case rotation_filter_kind::one_euro:
            return std::make_unique<one_euro_rotation_filter>(cfg.one_euro);
        }
        return std::make_unique<one_euro_rotation_filter>(cfg.one_euro);
    }

} // namespace pose

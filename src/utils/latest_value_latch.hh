#pragma once
#include <mutex>
#include <utility>

namespace utils
{
    // Hands a producer's latest value to a consumer across a thread boundary. A publish overwrites
    // whatever has not been taken, so a slow consumer gets the newest value and misses the ones
    // between; a take gets one whole value or nothing, never a mix of two.
    template <typename Payload>
    class latest_value_latch
    {
    public:
        void publish(Payload payload)
        {
            std::scoped_lock lk{ _mtx };
            _payload = std::move(payload);
            _unread = true;
        }

        // False when nothing has been published since the last take, leaving `out` untouched.
        bool try_take(Payload& out)
        {
            std::scoped_lock lk{ _mtx };
            if (!_unread) { return false; }
            _unread = false;
            out = _payload;
            return true;
        }

        // Reads the newest value without claiming it, which is what a readout does. `fn` runs under
        // the lock, so it returns a copy of what it wants, never a handle to it.
        template <typename Fn>
        auto read(Fn&& fn) const
        {
            std::scoped_lock lk{ _mtx };
            return fn(_payload);
        }

    private:
        mutable std::mutex _mtx;
        Payload _payload{};
        bool _unread{ false }; // a value has been published that no take has claimed
    };

} // namespace utils

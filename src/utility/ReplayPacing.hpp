#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>

namespace dai {
namespace replay {

using PlaybackClock = std::chrono::steady_clock;

inline PlaybackClock::duration playbackPeriod(double fps) {
    if(!std::isfinite(fps) || fps <= 0) throw std::invalid_argument("Replay invalid fps");
    using Duration = PlaybackClock::duration;
    const long double ticks = static_cast<long double>(Duration::period::den) / Duration::period::num / fps;
    if(ticks >= static_cast<long double>(Duration::max().count())) return Duration::max();
    return Duration(std::max<Duration::rep>(1, static_cast<Duration::rep>(ticks)));
}

// PC-1..PC-6: decode first, then wait before sending. T is the actual send
// start, C is the time every consumer accepted it. A late frame moves the
// schedule forward; no frame is skipped or sent early to recover lost time.
class PlaybackPacer {
   public:
    using Time = PlaybackClock::time_point;
    using Duration = PlaybackClock::duration;

    explicit PlaybackPacer(float fps = 0) {
        if(!std::isfinite(fps) || fps < 0) throw std::invalid_argument("Replay invalid fps");
        if(fps > 0) fixedInterval = playbackPeriod(fps);
    }

    template <typename Running>
    std::optional<Time> waitToSend(std::optional<Duration> recordedInterval, Running running) {
        return waitToSend(recordedInterval, running, [] { return PlaybackClock::now(); }, [](Duration delay) { std::this_thread::sleep_for(delay); });
    }

    // Clock and sleeper are supplied by deterministic policy tests as well as
    // the real-clock overload above; there is only one scheduling implementation.
    template <typename Running, typename Now, typename Sleep>
    std::optional<Time> waitToSend(std::optional<Duration> recordedInterval, Running running, Now now, Sleep sleep) {
        const auto interval = fixedInterval.value_or(recordedInterval && *recordedInterval > Duration::zero() ? *recordedInterval : playbackPeriod(30));
        auto target = now();
        if(started) {
            const auto next = sendStarted > Time::max() - interval ? Time::max() : sendStarted + interval;
            target = std::max(next, sendCompleted);
        }
        while(running()) {
            const auto current = now();
            if(current >= target) {
                sendStarted = current;
                started = true;
                return current;
            }
            sleep(std::min(target - current, std::chrono::duration_cast<Duration>(std::chrono::milliseconds(50))));
        }
        return std::nullopt;
    }

    void sent(Time completed = PlaybackClock::now()) {
        sendCompleted = completed;
    }

   private:
    std::optional<Duration> fixedInterval;
    bool started = false;
    Time sendStarted{};
    Time sendCompleted{};
};

}  // namespace replay
}  // namespace dai

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
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

// Frame n is due at the first send plus the sum of the intervals so far. A sleep that wakes late
// does not move that grid, so one frame still lands in each interval and the rate does not drift.
// A frame that is already due is sent immediately.
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
        if(!haveOrigin) return now();
        scheduled = addBounded(scheduled, interval);
        const auto target = dueAt();
        while(running()) {
            const auto at = now();
            if(at >= target) return at;
            sleep(std::min(target - at, std::chrono::duration_cast<Duration>(std::chrono::milliseconds(50))));
        }
        return std::nullopt;
    }

    void sent(Time completed = PlaybackClock::now()) {
        if(haveOrigin) return;
        origin = completed;
        haveOrigin = true;
    }

   private:
    static Duration addBounded(Duration base, Duration span) {
        if(span <= Duration::zero()) return base;
        if(base > Duration::max() - span) return Duration::max();
        return base + span;
    }

    Time dueAt() const {
        if(origin > Time::max() - scheduled) return Time::max();
        return origin + scheduled;
    }

    std::optional<Duration> fixedInterval;
    bool haveOrigin = false;
    Duration scheduled{};
    Time origin{};
};

}  // namespace replay
}  // namespace dai

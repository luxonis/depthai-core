#pragma once

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

template <class Clock = std::chrono::steady_clock>
class ClockOffsetSynchronizer {
   public:
    std::chrono::nanoseconds offset();

    void shutdown();

    virtual ~ClockOffsetSynchronizer();

   private:
    using ReferenceClockReader = std::function<typename Clock::time_point()>;

   protected:
    explicit ClockOffsetSynchronizer(ReferenceClockReader reader);
    ClockOffsetSynchronizer(const ClockOffsetSynchronizer&) = delete;
    ClockOffsetSynchronizer& operator=(const ClockOffsetSynchronizer&) = delete;

    std::chrono::nanoseconds measureOffset();
    std::chrono::nanoseconds computeMedian(const std::deque<std::chrono::nanoseconds>& samples);

    static constexpr size_t SAMPLING_WINDOW_SIZE = 10;  // Number of samples used for the rolling median.

    ReferenceClockReader referenceClockReader;
    std::atomic<bool> running;
    std::thread worker;
    std::mutex mtx;
    std::mutex cvmtx;
    std::condition_variable cv;

    std::deque<std::chrono::nanoseconds> offsets;
    std::chrono::nanoseconds median{0};  // Current median offset between steady clock and reference clock.
};

template class ClockOffsetSynchronizer<std::chrono::steady_clock>;
template class ClockOffsetSynchronizer<std::chrono::system_clock>;

class SteadyToRawOffset final : public ClockOffsetSynchronizer<std::chrono::steady_clock> {
   public:
    static SteadyToRawOffset& instance();

private:
    SteadyToRawOffset();
};

class SteadyToQTimerOffset final : public ClockOffsetSynchronizer<std::chrono::steady_clock> {
   public:
    static SteadyToQTimerOffset& instance();

private:
    SteadyToQTimerOffset();
};

class SystemToQTimerOffset final : public ClockOffsetSynchronizer<std::chrono::system_clock> {
public:
    static SystemToQTimerOffset& instance();

private:
    SystemToQTimerOffset();
};

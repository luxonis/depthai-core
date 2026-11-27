#include "ClockOffset.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <utility>
#include <vector>

#include "ErrorMacros.hpp"

namespace {

constexpr uint64_t CAM_IFE_CSID_QTIMER_MUL_FACTOR = 10000;
constexpr uint64_t CAM_IFE_CSID_QTIMER_DIV_FACTOR = 192;

// Convert a raw timespec into a steady_clock::time_point by reusing the steady clock's duration type.
std::chrono::steady_clock::time_point timespecToSteady(const timespec& ts) {
    using namespace std::chrono;
    const auto rawDuration = seconds(ts.tv_sec) + nanoseconds(ts.tv_nsec);
    const auto steadyDuration = duration_cast<std::chrono::steady_clock::duration>(rawDuration);
    return std::chrono::steady_clock::time_point(steadyDuration);
}

// Read the system CLOCK_MONOTONIC_RAW counter as a steady clock point.
std::chrono::steady_clock::time_point readMonotonicRawClock() {
    timespec tsRaw{};
    clock_gettime(CLOCK_MONOTONIC_RAW, &tsRaw);
    return timespecToSteady(tsRaw);
}

// QTimer helper: convert raw ticks into nanoseconds using the CAM IFE conversion constants.
uint64_t qtimerTicksToNs(uint64_t ticks) {
    return (ticks * CAM_IFE_CSID_QTIMER_MUL_FACTOR / CAM_IFE_CSID_QTIMER_DIV_FACTOR);
}

// Access the aarch64 virtual counter register (cntvct_el0) to read HW QTimer ticks.
uint64_t qtimerGetTicks() {
    unsigned long long val = 0;
    asm volatile("mrs %0, cntvct_el0" : "=r"(val));
    return val;
}

// Read the HW QTimer value and convert it into a Clock::time_point.
template <typename Clock = std::chrono::steady_clock>
typename Clock::time_point readQTimerClock() {
    using namespace std::chrono;
    const uint64_t nsSinceStart = qtimerTicksToNs(qtimerGetTicks());
    const auto duration = duration_cast<typename Clock::duration>(nanoseconds(nsSinceStart));
    return typename Clock::time_point(duration);
}

}  // namespace

template <typename Clock>
ClockOffsetSynchronizer<Clock>::ClockOffsetSynchronizer(ReferenceClockReader reader) : referenceClockReader(std::move(reader)), running(true) {
    // Take initial measurements to populate the window and compute the starting median.
    for(size_t i = 0; i < SAMPLING_WINDOW_SIZE; ++i) {
        offsets.push_back(measureOffset());
    }
    median = computeMedian(offsets);

    // Start background update thread
    worker = std::thread([this] {
        while(running.load()) {
            auto off = measureOffset();

            {
                std::lock_guard<std::mutex> lock(mtx);
                offsets.push_back(off);
                if(offsets.size() > SAMPLING_WINDOW_SIZE) offsets.pop_front();
                median = computeMedian(offsets);
            }

            // Wait 20ms, but support early shutdown
            std::unique_lock<std::mutex> lock(cvmtx);
            cv.wait_for(lock, std::chrono::milliseconds(20), [this] { return !running.load(); });
        }
    });
}

template <typename Clock>
ClockOffsetSynchronizer<Clock>::~ClockOffsetSynchronizer() {
    shutdown();
}

template <typename Clock>
void ClockOffsetSynchronizer<Clock>::shutdown() {
    running.store(false);
    cv.notify_all();
    if(worker.joinable()) worker.join();
}

template <typename Clock>
std::chrono::nanoseconds ClockOffsetSynchronizer<Clock>::offset() {
    std::lock_guard<std::mutex> lock(mtx);
    return median;
}

template <typename Clock>
std::chrono::nanoseconds ClockOffsetSynchronizer<Clock>::measureOffset() {
    using namespace std::chrono;
    auto clockNow = Clock::now();
    auto referenceNow = referenceClockReader();
    return clockNow - referenceNow;
}

template <typename Clock>
std::chrono::nanoseconds ClockOffsetSynchronizer<Clock>::computeMedian(const std::deque<std::chrono::nanoseconds>& samples) {
    DAI_CHECK_IN(!samples.empty());  // Ensure there are samples to compute median, should never be triggered
    // Needs a copy because std::nth_element raranges the elements in place
    std::vector<std::chrono::nanoseconds> copy(samples.begin(), samples.end());
    size_t mid = copy.size() / 2;
    std::nth_element(copy.begin(), copy.begin() + mid, copy.end());
    if(copy.size() % 2 == 1) {
        return copy[mid];
    } else {
        auto lower = *std::max_element(copy.begin(), copy.begin() + mid);
        return (lower + copy[mid]) / 2;
    }
}

SteadyToRawOffset& SteadyToRawOffset::instance() {
    static SteadyToRawOffset inst;
    return inst;
}

SteadyToRawOffset::SteadyToRawOffset() : ClockOffsetSynchronizer(readMonotonicRawClock) {}

SteadyToQTimerOffset& SteadyToQTimerOffset::instance() {
    static SteadyToQTimerOffset inst;
    return inst;
}

SteadyToQTimerOffset::SteadyToQTimerOffset() : ClockOffsetSynchronizer(readQTimerClock<std::chrono::steady_clock>) {}

SystemToQTimerOffset& SystemToQTimerOffset::instance() {
    static SystemToQTimerOffset inst;
    return inst;
}

SystemToQTimerOffset::SystemToQTimerOffset() : ClockOffsetSynchronizer(readQTimerClock<std::chrono::system_clock>) {}

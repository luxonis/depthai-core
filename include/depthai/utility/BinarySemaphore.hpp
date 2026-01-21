#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace dai {
namespace utility {

/**
 * @brief A binary semaphore implementation
 * Should be removed and replaced with std::binary_semaphore if and when C++20 is supported
 */
class BinarySemaphore {
    std::mutex mtx;
    std::condition_variable cv;
    bool available = false;  // zero-initialized

   public:
    BinarySemaphore() = default;
    explicit BinarySemaphore(bool initiallyAvailable) : available(initiallyAvailable) {}

    // signal / post / V
    void release() {
        {
            std::lock_guard<std::mutex> lk(mtx);
            available = true;
        }
        cv.notify_one();
    }

    // wait / P
    void acquire() {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait(lk, [&] { return available; });
        available = false;  // consume
    }

    // try-wait
    bool tryAcquire() {
        std::lock_guard<std::mutex> lk(mtx);
        if(!available) return false;
        available = false;
        return true;
    }

    template <class Rep, class Period>
    bool tryAcquireFor(const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lk(mtx);
        if(!cv.wait_for(lk, timeout, [&] { return available; })) return false;
        available = false;
        return true;
    }

    template <class Clock, class Duration>
    bool tryAcquireUntil(const std::chrono::time_point<Clock, Duration>& deadline) {
        std::unique_lock<std::mutex> lk(mtx);
        if(!cv.wait_until(lk, deadline, [&] { return available; })) return false;
        available = false;  // consume
        return true;
    }
};

}  // namespace utility
}  // namespace dai

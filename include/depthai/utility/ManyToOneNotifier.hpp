#pragma once

#include <atomic>
#include <chrono>
#include <stdexcept>

#include "BinarySemaphore.hpp"

namespace dai {
namespace utility {

class ManyToOneNotifier {
    BinarySemaphore semaphore{false};
    std::atomic<bool> waiting{false};

   public:
    ManyToOneNotifier() = default;

    /**
     * @brief Notify the waiting thread
     * Should always be called after the state change that the waiting thread is waiting for
     */
    void notifyOne() {
        semaphore.release();
    }

    /**
     * @brief Wait until notified
     * @arg pred Predicate to check after each notification - IMPORTANT: must be thread-safe
     */
    template <typename Pred>
    void wait(Pred pred) {
        if(waiting.exchange(true)) {
            throw std::runtime_error("ManyToOneNotifier: Multiple threads waiting on the same notifier is not supported");
        }
        try {
            while(!pred()) semaphore.acquire();
        } catch(...) {
            waiting = false;
            throw;
        }
        waiting = false;
    }

    /**
     * @brief Wait until notified or timeout occurs
     * @arg pred Predicate to check after each notification - IMPORTANT: must be thread-safe
     */
    template <typename Pred, typename Rep, typename Period>
    bool waitFor(Pred pred, std::chrono::duration<Rep, Period> timeout) {
        if(waiting.exchange(true)) {
            throw std::runtime_error("ManyToOneNotifier: Multiple threads waiting on the same notifier is not supported");
        }

        try {
            auto deadline = std::chrono::steady_clock::now() + timeout;

            while(!pred()) {
                if(!semaphore.tryAcquireUntil(deadline)) {
                    return pred();
                }
            }
            return true;
        } catch(...) {
            waiting = false;
            throw;
        }
        waiting = false;
    }
};

}  // namespace utility
}  // namespace dai

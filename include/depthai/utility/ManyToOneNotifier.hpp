#pragma once

#include <atomic>
#include <chrono>
#include <stdexcept>

#include "BinarySemaphore.hpp"

namespace dai {
namespace utility {

/**
 * @class ManyToOneNotifier
 * @brief Used to notify a single waiting thread from multiple notifying threads.
 * Not thread-safe for multiple waiting threads.
 * Should not be used if there is another thread waiting on any of the notifying queues.
 */
class ManyToOneNotifier {
    BinarySemaphore semaphore{false};
    std::atomic<bool> waiting{false};

   public:
    ManyToOneNotifier() = default;

    /**
     * @brief Notify the waiting thread
     * Should always be called after any state change that the waiting thread is waiting for
     */
    void notifyOne() {
        semaphore.release();
    }

    /**
     * @brief Wait until notified
     * If the predicate consists of multiple conditions connected by AND, it should be ensured that the state does not change between the conditions.
     * This can be achieved either by using a mutex or by ensuring that the condition can only be made false by the waiting thread (e.g. notifying thread fills queues and the waiting thread empties them).
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
     * If the predicate consists of multiple conditions connected by AND, it should be ensured that the state does not change between the conditions.
     * This can be achieved either by using a mutex or by ensuring that the condition can only be made false by the waiting thread (e.g. notifying thread fills queues and the waiting thread empties them).
     * @arg pred Predicate to check after each notification - IMPORTANT: must be thread-safe
     */
    template <typename Pred, typename Rep, typename Period>
    bool waitFor(Pred pred, std::chrono::duration<Rep, Period> timeout) {
        if(waiting.exchange(true)) {
            throw std::runtime_error("ManyToOneNotifier: Multiple threads waiting on the same notifier is not supported");
        }

        struct WaitingGuard {
            std::atomic<bool>& waitingRef;
            ~WaitingGuard() { waitingRef = false; }
        } guard{waiting};

        try {
            auto deadline = std::chrono::steady_clock::now() + timeout;

            while(!pred()) {
                if(!semaphore.tryAcquireUntil(deadline)) {
                    return pred();
                }
            }
            return true;
        } catch(...) {
            throw;
        }
    }
};

}  // namespace utility
}  // namespace dai

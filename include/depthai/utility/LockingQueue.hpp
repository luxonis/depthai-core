#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <type_traits>
#include <utility>
#include <vector>

namespace dai {

class MessageQueue;

namespace detail {
// Scalar checkpoints only; no recorder locks/callbacks are introduced under the queue guard.
struct QueuePushTiming {
    bool enabled = false;
    std::chrono::steady_clock::time_point lockWaitStartedAt{};
    std::chrono::steady_clock::time_point lockAcquiredAt{};
    std::chrono::steady_clock::time_point capacityWaitStartedAt{};
    std::chrono::steady_clock::time_point capacityWaitFinishedAt{};
    std::chrono::steady_clock::time_point enqueueStartedAt{};
    std::chrono::steady_clock::time_point guardReleasedAt{};
    std::chrono::steady_clock::time_point notifyStartedAt{};
    std::chrono::steady_clock::time_point notifyFinishedAt{};
};

enum class QueuePopOutcome { UNKNOWN, POPPED, TIMEOUT, CLOSED };
struct QueuePopTiming {
    std::chrono::steady_clock::time_point lockWaitStartedAt{};
    std::chrono::steady_clock::time_point lockAcquiredAt{};
    std::chrono::steady_clock::time_point waitStartedAt{};
    std::chrono::steady_clock::time_point waitFinishedAt{};
    std::chrono::steady_clock::time_point popCompletedAt{};
    std::chrono::steady_clock::time_point guardReleasedAt{};
    QueuePopOutcome outcome = QueuePopOutcome::UNKNOWN;
};

// Used only by the private test-reader bridge; no SDK object acquires new fields.
struct QueueReadTiming {
    std::chrono::steady_clock::time_point entry{};
    std::chrono::steady_clock::time_point beforePop{};
    std::chrono::steady_clock::time_point afterPop{};
    std::chrono::steady_clock::time_point castFinished{};
    std::chrono::steady_clock::time_point returnReady{};
    bool nullCast = false;
    QueuePopTiming pop;
};

// Private host diagnostics: collect identities while locked, inspect them only after push returns.
template <typename T>
struct QueuePushDiagnostics {
    std::vector<T> evicted;
    size_t evictionCount = 0;
    size_t captureErrors = 0;
    size_t sizeBefore = 0;
    size_t sizeAfter = 0;
    unsigned capacity = 0;
    bool blocking = false;
    bool discardedIncoming = false;
    std::chrono::steady_clock::time_point completedAt{};
    QueuePushTiming timing;
};
struct SyncDebugQueueAccess;
}  // namespace detail

// class Mutex : public std::mutex {
//    public:
//     using std::mutex::mutex;
//     Mutex() = default;
//     ~Mutex() = default;
//     Mutex(const Mutex&) : Mutex() {}
//     Mutex& operator=(const Mutex&) = delete;
//     Mutex(Mutex&&) : Mutex() {}
//     Mutex& operator=(Mutex&&) = delete;
// };

enum class LockingQueueState { SUCCESS, BLOCKED, CANCELLED };

template <typename T>
class LockingQueue {
   public:
    LockingQueue() = default;
    explicit LockingQueue(unsigned maxSize, bool blocking = true) {
        this->maxSize = maxSize;
        this->blocking = blocking;
    }
    LockingQueue(const LockingQueue& obj) : maxSize(obj.maxSize), blocking(obj.blocking), queue(obj.queue), destructed(obj.destructed){};
    LockingQueue(LockingQueue&& obj) noexcept : maxSize(obj.maxSize), blocking(obj.blocking), queue(std::move(obj.queue)), destructed(obj.destructed){};
    LockingQueue& operator=(const LockingQueue& obj) {
        maxSize = obj.maxSize;
        blocking = obj.blocking;
        queue = obj.queue;
        destructed = obj.destructed;
        return *this;
    }
    LockingQueue& operator=(LockingQueue&& obj) noexcept {
        maxSize = obj.maxSize;
        blocking = obj.blocking;
        queue = std::move(obj.queue);
        destructed = obj.destructed;
        return *this;
    }

    void setMaxSize(unsigned sz) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        maxSize = sz;
    }

    void setBlocking(bool bl) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        blocking = bl;
    }

    unsigned getMaxSize() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return maxSize;
    }

    unsigned getSize() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return queue.size();
    }

    unsigned isFull() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return queue.size() >= maxSize;
    }

    bool getBlocking() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return blocking;
    }

    void destruct() {
        std::unique_lock<std::mutex> lock(guard);
        if(!destructed) {
            signalPop.notify_all();
            signalPush.notify_all();
            destructed = true;
        }
    }

    bool isDestroyed() const {
        std::unique_lock<std::mutex> lock(guard);
        return destructed;
    }

    ~LockingQueue() = default;

    template <typename Rep, typename Period>
    bool waitAndConsumeAll(std::function<void(T&)> callback, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);

            // First checks predicate, then waits
            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

            // Continue here if and only if queue has any elements
            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool waitAndConsumeAll(std::function<void(T&)> callback) {
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return !queue.empty() || destructed; });
            if(queue.empty()) return false;
            if(destructed) return false;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool consumeAll(std::function<void(T&)> callback) {
        {
            std::lock_guard<std::mutex> lock(guard);

            if(queue.empty()) return false;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool push(T const& data, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        return pushImpl(data, std::move(callback), nullptr);
    }

    bool push(T&& data, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        return pushImpl(std::move(data), std::move(callback), nullptr);
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(
        T const& data, std::chrono::duration<Rep, Period> timeout, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        return tryWaitAndPushImpl(data, timeout, std::move(callback), nullptr);
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(
        T&& data, std::chrono::duration<Rep, Period> timeout, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        return tryWaitAndPushImpl(std::move(data), timeout, std::move(callback), nullptr);
    }

   private:
    friend struct detail::SyncDebugQueueAccess;

    struct PushTimingScope {
        detail::QueuePushDiagnostics<T>* diagnostics;
        explicit PushTimingScope(detail::QueuePushDiagnostics<T>* diagnostics) : diagnostics(diagnostics) {
            if(diagnostics && diagnostics->timing.enabled) diagnostics->timing.lockWaitStartedAt = std::chrono::steady_clock::now();
        }
        ~PushTimingScope() {
            // Declared before the lock, so this runs after unlocking, including early returns/exceptions.
            if(diagnostics && diagnostics->timing.enabled) diagnostics->timing.guardReleasedAt = std::chrono::steady_clock::now();
        }
    };

    void beginPushDiagnostics(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(!diagnostics) return;
        if(diagnostics->timing.enabled) diagnostics->timing.lockAcquiredAt = std::chrono::steady_clock::now();
        diagnostics->sizeBefore = queue.size();
        diagnostics->sizeAfter = queue.size();
        diagnostics->capacity = maxSize;
        diagnostics->blocking = blocking;
        diagnostics->discardedIncoming = maxSize == 0;
    }

    void capacityWaitDiagnostics(detail::QueuePushDiagnostics<T>* diagnostics, bool finished) {
        if(!diagnostics || !diagnostics->timing.enabled) return;
        auto& checkpoint = finished ? diagnostics->timing.capacityWaitFinishedAt : diagnostics->timing.capacityWaitStartedAt;
        checkpoint = std::chrono::steady_clock::now();
    }

    void enqueueDiagnostics(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(diagnostics && diagnostics->timing.enabled) diagnostics->timing.enqueueStartedAt = std::chrono::steady_clock::now();
    }

    void popEvicted(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(diagnostics) {
            ++diagnostics->evictionCount;
            // Only copy nothrow message handles; generic move-only queue types remain usable.
            if constexpr(std::is_nothrow_copy_constructible_v<T>) {
                try {
                    diagnostics->evicted.push_back(queue.front());
                } catch(...) {
                    // A failed diagnostic copy must not change queue delivery semantics.
                    ++diagnostics->captureErrors;
                }
            } else {
                ++diagnostics->captureErrors;
            }
        }
        queue.pop();
    }

    void endPushDiagnostics(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(!diagnostics) return;
        diagnostics->sizeAfter = queue.size();
        diagnostics->completedAt = std::chrono::steady_clock::now();
    }

    void notifyPush(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(diagnostics && diagnostics->timing.enabled) diagnostics->timing.notifyStartedAt = std::chrono::steady_clock::now();
        signalPush.notify_all();
        if(diagnostics && diagnostics->timing.enabled) diagnostics->timing.notifyFinishedAt = std::chrono::steady_clock::now();
    }

    bool pushImpl(T const& data, std::function<void(LockingQueueState, size_t)> callback, detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            PushTimingScope timingScope(diagnostics);
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
                enqueueDiagnostics(diagnostics);
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    popEvicted(diagnostics);
                }
                endPushDiagnostics(diagnostics);
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    popEvicted(diagnostics);
                }
            } else {
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                capacityWaitDiagnostics(diagnostics, false);
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                capacityWaitDiagnostics(diagnostics, true);
                if(destructed) return false;
            }

            enqueueDiagnostics(diagnostics);
            queue.push(data);
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        notifyPush(diagnostics);
        return true;
    }

    bool pushImpl(T&& data, std::function<void(LockingQueueState, size_t)> callback, detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            PushTimingScope timingScope(diagnostics);
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
                enqueueDiagnostics(diagnostics);
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    popEvicted(diagnostics);
                }
                endPushDiagnostics(diagnostics);
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    popEvicted(diagnostics);
                }
            } else {
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                capacityWaitDiagnostics(diagnostics, false);
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                capacityWaitDiagnostics(diagnostics, true);
                if(destructed) return false;
            }

            enqueueDiagnostics(diagnostics);
            queue.push(std::move(data));
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        notifyPush(diagnostics);
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPushImpl(T const& data,
                            std::chrono::duration<Rep, Period> timeout,
                            std::function<void(LockingQueueState, size_t)> callback,
                            detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            PushTimingScope timingScope(diagnostics);
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
                enqueueDiagnostics(diagnostics);
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    popEvicted(diagnostics);
                }
                endPushDiagnostics(diagnostics);
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    popEvicted(diagnostics);
                }
            } else {
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                // First checks predicate, then waits
                capacityWaitDiagnostics(diagnostics, false);
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                capacityWaitDiagnostics(diagnostics, true);
                if(!pred) {
                    callback(LockingQueueState::CANCELLED, queue.size());
                }
                if(!pred) return false;
                if(destructed) return false;
            }

            enqueueDiagnostics(diagnostics);
            queue.push(data);
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        notifyPush(diagnostics);
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPushImpl(T&& data,
                            std::chrono::duration<Rep, Period> timeout,
                            std::function<void(LockingQueueState, size_t)> callback,
                            detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            PushTimingScope timingScope(diagnostics);
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
                enqueueDiagnostics(diagnostics);
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    popEvicted(diagnostics);
                }
                endPushDiagnostics(diagnostics);
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    popEvicted(diagnostics);
                }
            } else {
                // First checks predicate, then waits
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                capacityWaitDiagnostics(diagnostics, false);
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                capacityWaitDiagnostics(diagnostics, true);
                if(!pred) {
                    callback(LockingQueueState::CANCELLED, queue.size());
                }
                if(!pred) return false;
                if(destructed) return false;
            }

            enqueueDiagnostics(diagnostics);
            queue.push(std::move(data));
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        notifyPush(diagnostics);
        return true;
    }

   public:
    bool empty() const {
        std::lock_guard<std::mutex> lock(guard);
        return queue.empty();
    }

    bool front(T& value) {
        std::unique_lock<std::mutex> lock(guard);
        if(queue.empty()) {
            return false;
        }

        value = queue.front();
        return true;
    }

    bool tryPop(T& value) {
        {
            std::lock_guard<std::mutex> lock(guard);
            if(queue.empty()) {
                return false;
            }

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    bool waitAndPop(T& value) {
        return waitAndPopImpl(value, nullptr);
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPop(T& value, std::chrono::duration<Rep, Period> timeout) {
        return tryWaitAndPopImpl(value, timeout, nullptr);
    }

    void waitEmpty() {
        std::unique_lock<std::mutex> lock(guard);
        signalPop.wait(lock, [this]() { return queue.empty() || destructed; });
    }

   private:
    friend class MessageQueue;

    struct PopTimingScope {
        detail::QueuePopTiming* timing;
        explicit PopTimingScope(detail::QueuePopTiming* timing) : timing(timing) {
            if(timing) timing->lockWaitStartedAt = std::chrono::steady_clock::now();
        }
        ~PopTimingScope() {
            // Runs after guard destruction, also on timeout/closure/unwinding.
            if(timing && timing->lockAcquiredAt != std::chrono::steady_clock::time_point{}) timing->guardReleasedAt = std::chrono::steady_clock::now();
        }
    };

    bool waitAndPopImpl(T& value, detail::QueuePopTiming* timing) {
        {
            PopTimingScope timingScope(timing);
            std::unique_lock<std::mutex> lock(guard);
            if(timing) timing->lockAcquiredAt = std::chrono::steady_clock::now();

            if(timing) timing->waitStartedAt = std::chrono::steady_clock::now();
            signalPush.wait(lock, [this]() { return (!queue.empty() || destructed); });
            if(timing) timing->waitFinishedAt = std::chrono::steady_clock::now();
            if(queue.empty() || destructed) {
                if(timing) timing->outcome = detail::QueuePopOutcome::CLOSED;
                return false;
            }

            value = std::move(queue.front());
            queue.pop();
            if(timing) {
                timing->popCompletedAt = std::chrono::steady_clock::now();
                timing->outcome = detail::QueuePopOutcome::POPPED;
            }
        }
        signalPop.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPopImpl(T& value, std::chrono::duration<Rep, Period> timeout, detail::QueuePopTiming* timing) {
        {
            PopTimingScope timingScope(timing);
            std::unique_lock<std::mutex> lock(guard);
            if(timing) timing->lockAcquiredAt = std::chrono::steady_clock::now();

            // First checks predicate, then waits
            if(timing) timing->waitStartedAt = std::chrono::steady_clock::now();
            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(timing) {
                timing->waitFinishedAt = std::chrono::steady_clock::now();
                timing->outcome = !pred ? detail::QueuePopOutcome::TIMEOUT : (destructed ? detail::QueuePopOutcome::CLOSED : detail::QueuePopOutcome::UNKNOWN);
            }
            if(!pred) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
            if(timing) {
                timing->popCompletedAt = std::chrono::steady_clock::now();
                timing->outcome = detail::QueuePopOutcome::POPPED;
            }
        }
        signalPop.notify_all();
        return true;
    }

    unsigned maxSize = std::numeric_limits<unsigned>::max();
    bool blocking = true;
    std::queue<T> queue;
    mutable std::mutex guard;
    bool destructed{false};
    std::condition_variable signalPop;
    std::condition_variable signalPush;
};

}  // namespace dai

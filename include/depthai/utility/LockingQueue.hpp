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

namespace detail {
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

    void beginPushDiagnostics(detail::QueuePushDiagnostics<T>* diagnostics) {
        if(!diagnostics) return;
        diagnostics->sizeBefore = queue.size();
        diagnostics->sizeAfter = queue.size();
        diagnostics->capacity = maxSize;
        diagnostics->blocking = blocking;
        diagnostics->discardedIncoming = maxSize == 0;
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

    bool pushImpl(T const& data, std::function<void(LockingQueueState, size_t)> callback, detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
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
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
            }

            queue.push(data);
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    bool pushImpl(T&& data, std::function<void(LockingQueueState, size_t)> callback, detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
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
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
            }

            queue.push(std::move(data));
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPushImpl(T const& data,
                            std::chrono::duration<Rep, Period> timeout,
                            std::function<void(LockingQueueState, size_t)> callback,
                            detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
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
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                if(!pred) {
                    callback(LockingQueueState::CANCELLED, queue.size());
                }
                if(!pred) return false;
                if(destructed) return false;
            }

            queue.push(data);
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPushImpl(T&& data,
                            std::chrono::duration<Rep, Period> timeout,
                            std::function<void(LockingQueueState, size_t)> callback,
                            detail::QueuePushDiagnostics<T>* diagnostics) {
        {
            std::unique_lock<std::mutex> lock(guard);
            beginPushDiagnostics(diagnostics);
            if(maxSize == 0) {
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
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                if(!pred) {
                    callback(LockingQueueState::CANCELLED, queue.size());
                }
                if(!pred) return false;
                if(destructed) return false;
            }

            queue.push(std::move(data));
            endPushDiagnostics(diagnostics);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
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
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return (!queue.empty() || destructed); });
            if(queue.empty()) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPop(T& value, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);

            // First checks predicate, then waits
            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    void waitEmpty() {
        std::unique_lock<std::mutex> lock(guard);
        signalPop.wait(lock, [this]() { return queue.empty() || destructed; });
    }

   private:
    unsigned maxSize = std::numeric_limits<unsigned>::max();
    bool blocking = true;
    std::queue<T> queue;
    mutable std::mutex guard;
    bool destructed{false};
    std::condition_variable signalPop;
    std::condition_variable signalPush;
};

}  // namespace dai

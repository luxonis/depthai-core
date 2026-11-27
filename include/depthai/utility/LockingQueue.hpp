#pragma once
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>

namespace dai {

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
    LockingQueue(const LockingQueue& obj)
        : maxSize(obj.maxSize), blocking(obj.blocking), queue(obj.queue), destructed(obj.destructed), overflowSlots(obj.overflowSlots){};
    LockingQueue(LockingQueue&& obj) noexcept
        : maxSize(obj.maxSize), blocking(obj.blocking), queue(std::move(obj.queue)), destructed(obj.destructed), overflowSlots(obj.overflowSlots){};
    LockingQueue& operator=(const LockingQueue& obj) {
        maxSize = obj.maxSize;
        blocking = obj.blocking;
        queue = obj.queue;
        destructed = obj.destructed;
        overflowSlots = obj.overflowSlots;
        return *this;
    }
    LockingQueue& operator=(LockingQueue&& obj) noexcept {
        maxSize = obj.maxSize;
        blocking = obj.blocking;
        queue = std::move(obj.queue);
        destructed = obj.destructed;
        overflowSlots = obj.overflowSlots;
        return *this;
    }

    void setMaxSize(unsigned sz) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        maxSize = sz;
        signalPop.notify_all();
    }

    void setBlocking(bool bl) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        blocking = bl;
        signalPop.notify_all();
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

    // Let one later push enqueue while size >= maxSize. maxSize itself is unchanged.
    void allowOneOverflow() {
        std::unique_lock<std::mutex> lock(guard);
        ++overflowSlots;
        signalPop.notify_all();
    }

    void clearOverflow() {
        std::unique_lock<std::mutex> lock(guard);
        overflowSlots = 0;
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
            signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(queue.empty()) return false;

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
        {
            std::unique_lock<std::mutex> lock(guard);
            if(destructed) return false;
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(blocking && queue.size() >= maxSize) {
                callback(LockingQueueState::BLOCKED, queue.size());
            }
            bool cancelled = false;
            if(!preparePush(
                   lock,
                   [this](std::unique_lock<std::mutex>& held) {
                       signalPop.wait(held, [this]() { return queue.size() < maxSize || destructed || !blocking || overflowSlots > 0; });
                       return true;
                   },
                   cancelled)) {
                if(cancelled) callback(LockingQueueState::CANCELLED, queue.size());
                return false;
            }

            queue.push(data);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    bool push(T&& data, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(destructed) return false;
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(blocking && queue.size() >= maxSize) {
                callback(LockingQueueState::BLOCKED, queue.size());
            }
            bool cancelled = false;
            if(!preparePush(
                   lock,
                   [this](std::unique_lock<std::mutex>& held) {
                       signalPop.wait(held, [this]() { return queue.size() < maxSize || destructed || !blocking || overflowSlots > 0; });
                       return true;
                   },
                   cancelled)) {
                if(cancelled) callback(LockingQueueState::CANCELLED, queue.size());
                return false;
            }

            queue.push(std::move(data));

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(
        T const& data, std::chrono::duration<Rep, Period> timeout, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(destructed) return false;
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(blocking && queue.size() >= maxSize) {
                callback(LockingQueueState::BLOCKED, queue.size());
            }
            bool cancelled = false;
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            if(!preparePush(
                   lock,
                   [this, deadline](std::unique_lock<std::mutex>& held) {
                       return signalPop.wait_until(held, deadline, [this]() { return queue.size() < maxSize || destructed || !blocking || overflowSlots > 0; });
                   },
                   cancelled)) {
                if(cancelled) callback(LockingQueueState::CANCELLED, queue.size());
                return false;
            }

            queue.push(data);

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(
        T&& data, std::chrono::duration<Rep, Period> timeout, std::function<void(LockingQueueState, size_t)> callback = [](LockingQueueState, size_t) {}) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(destructed) return false;
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(blocking && queue.size() >= maxSize) {
                callback(LockingQueueState::BLOCKED, queue.size());
            }
            bool cancelled = false;
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            if(!preparePush(
                   lock,
                   [this, deadline](std::unique_lock<std::mutex>& held) {
                       return signalPop.wait_until(held, deadline, [this]() { return queue.size() < maxSize || destructed || !blocking || overflowSlots > 0; });
                   },
                   cancelled)) {
                if(cancelled) callback(LockingQueueState::CANCELLED, queue.size());
                return false;
            }

            queue.push(std::move(data));

            callback(LockingQueueState::SUCCESS, queue.size());
        }
        signalPush.notify_all();
        return true;
    }

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
            signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(queue.empty()) return false;

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
    // Caller holds guard. Returns false when the queue is destroyed or the wait times out.
    // A true return means one element may be enqueued. One overflow slot is consumed only
    // when the queue is still blocking and already at maxSize.
    template <typename WaitFn>
    bool preparePush(std::unique_lock<std::mutex>& lock, WaitFn waitFn, bool& cancelled) {
        cancelled = false;
        while(true) {
            if(destructed) return false;
            if(!blocking) {
                while(queue.size() >= maxSize) {
                    queue.pop();
                }
                return true;
            }
            if(queue.size() < maxSize) return true;
            if(overflowSlots > 0) {
                --overflowSlots;
                return true;
            }
            if(!waitFn(lock)) {
                if(destructed || !blocking || queue.size() < maxSize || overflowSlots > 0) continue;
                cancelled = true;
                return false;
            }
        }
    }

    unsigned maxSize = std::numeric_limits<unsigned>::max();
    bool blocking = true;
    std::queue<T> queue;
    mutable std::mutex guard;
    bool destructed{false};
    std::condition_variable signalPop;
    std::condition_variable signalPush;
    unsigned overflowSlots{0};
};

}  // namespace dai

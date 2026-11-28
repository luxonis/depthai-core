#pragma once
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>

namespace dai {

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
        std::unique_lock<std::mutex> lock(guard);
        maxSize = sz;
    }

    void setBlocking(bool bl) {
        std::unique_lock<std::mutex> lock(guard);
        blocking = bl;
    }

    unsigned getMaxSize() const {
        std::unique_lock<std::mutex> lock(guard);
        return maxSize;
    }

    unsigned getSize() const {
        std::unique_lock<std::mutex> lock(guard);
        return queue.size();
    }

    unsigned isFull() const {
        std::unique_lock<std::mutex> lock(guard);
        return queue.size() >= maxSize;
    }

    bool getBlocking() const {
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

            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

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
        {
            std::unique_lock<std::mutex> lock(guard);
            if(maxSize == 0) {
                while(!queue.empty()) {  // maxSize can change after elements were queued
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                while(queue.size() >= maxSize) {  // Drop the oldest until the next one fits; maxSize can shrink
                    queue.pop();
                }
            } else {
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
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
            if(maxSize == 0) {
                while(!queue.empty()) {  // maxSize can change after elements were queued
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                while(queue.size() >= maxSize) {  // Drop the oldest until the next one fits; maxSize can shrink
                    queue.pop();
                }
            } else {
                if(queue.size() >= maxSize) {
                    callback(LockingQueueState::BLOCKED, queue.size());
                }
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
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
            if(maxSize == 0) {
                while(!queue.empty()) {  // maxSize can change after elements were queued
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                while(queue.size() >= maxSize) {  // Drop the oldest until the next one fits; maxSize can shrink
                    queue.pop();
                }
            } else {
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
            if(maxSize == 0) {
                while(!queue.empty()) {  // maxSize can change after elements were queued
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                while(queue.size() >= maxSize) {  // Drop the oldest until the next one fits; maxSize can shrink
                    queue.pop();
                }
            } else {
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

            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
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

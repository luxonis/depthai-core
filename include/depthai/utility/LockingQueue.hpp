#pragma once
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>

template <typename T>
class LockingQueue {
   public:
    LockingQueue() = default;
    explicit LockingQueue(int maxSize, bool overwrite = false) {
        this->maxSize = maxSize;
        this->overwrite = overwrite;
    }

    void destruct() {
        destructed = true;
        signalPop.notify_all();
        signalPush.notify_all();
    }
    ~LockingQueue() = default;

    void waitAndConsumeAll(std::function<void(T&)> callback) {
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return !queue.empty() || destructed; });
            if(destructed) return;

            if(queue.empty()) return;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
    }

    void consumeAll(std::function<void(T&)> callback) {
        {
            std::lock_guard<std::mutex> lock(guard);

            if(queue.empty()) return;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
    }

    bool push(T const& data) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(overwrite) {
                if(queue.size() >= maxSize) {
                    queue.pop();
                }
            } else {
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
            }

            queue.push(data);
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(T const& data, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(overwrite) {
                if(queue.size() >= maxSize) {
                    queue.pop();
                }
            } else {
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                if(!pred) return false;
                if(destructed) return false;
            }

            queue.push(data);
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

            value = queue.front();
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    bool waitAndPop(T& value) {
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return (!queue.empty() || destructed); });
            if(destructed) return false;
            if(queue.empty()) return false;

            value = queue.front();
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
            if(destructed) return false;
            if(!pred) return false;

            value = queue.front();
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
    bool overwrite = false;
    std::queue<T> queue;
    mutable std::mutex guard;
    std::atomic<bool> destructed{false};
    std::condition_variable signalPop;
    std::condition_variable signalPush;
};
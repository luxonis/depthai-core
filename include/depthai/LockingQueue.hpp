#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>

template<typename T>
class LockingQueue
{
public:
    LockingQueue() = default;
    LockingQueue(int maxsize){
        this->maxsize = maxsize;
    }


    void waitAndConsumeAll(std::function<void(T&)> callback){
        std::unique_lock<std::mutex> lock(guard);
        while (queue.empty())
        {
            signal.wait(lock);
        }
        
        while(!queue.empty()){
            callback(queue.front());
            queue.pop();
        }
    }

    void consumeAll(std::function<void(T&)> callback){
        std::lock_guard<std::mutex> lock(guard);
        while(!queue.empty()){
            callback(queue.front());
            queue.pop();
        }
    }

    bool push(T const& _data)
    {
        {
            std::lock_guard<std::mutex> lock(guard);
            if(queue.size() >= maxsize){
                return false;
            }
            queue.push(_data);
        }
        signal.notify_one();
        return true;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(guard);
        return queue.empty();
    }

    bool tryPop(T& _value)
    {
        std::lock_guard<std::mutex> lock(guard);
        if (queue.empty())
        {
            return false;
        }

        _value = queue.front();
        queue.pop();
        return true;
    }

    void waitAndPop(T& _value)
    {
        std::unique_lock<std::mutex> lock(guard);
        while (queue.empty())
        {
            signal.wait(lock);
        }

        _value = queue.front();
        queue.pop();
    }

    bool tryWaitAndPop(T& _value, int _milli)
    {
        std::unique_lock<std::mutex> lock(guard);
        while (queue.empty())
        {
            signal.wait_for(lock, std::chrono::milliseconds(_milli));
            return false;
        }

        _value = queue.front();
        queue.pop();
        return true;
    }

private:
    int maxsize = 0;
    std::queue<T> queue;
    mutable std::mutex guard;
    std::condition_variable signal;
};
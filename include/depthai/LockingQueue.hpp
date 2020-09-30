#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <limits>

template<typename T>
class LockingQueue
{
public:
    LockingQueue() = default;
    LockingQueue(int maxsize){
        this->maxsize = maxsize;
    }


    void waitAndConsumeAll(std::function<void(T&)> callback){
        
        {
            std::unique_lock<std::mutex> lock(guard);
            while (queue.empty())
            {
                signalPush.wait(lock);
            }

            if (queue.empty()) return;
            
            while(!queue.empty()){
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();

    }

    void consumeAll(std::function<void(T&)> callback){
        {
            std::lock_guard<std::mutex> lock(guard);

            if (queue.empty()) return;

            while(!queue.empty()){
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        
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
        signalPush.notify_all();
        return true;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(guard);
        return queue.empty();
    }


    bool front(T& _value)
    {
        std::unique_lock<std::mutex> lock(guard);
        if (queue.empty()){
            return false;
        }

        _value = queue.front();
        return true;
    }

    bool tryPop(T& _value)
    {
        {
            std::lock_guard<std::mutex> lock(guard);
            if (queue.empty())
            {
                return false;
            }

            _value = queue.front();
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    void waitAndPop(T& _value)
    {
        {
            std::unique_lock<std::mutex> lock(guard);
            while (queue.empty())
            {
                signalPush.wait(lock);
            }

            _value = queue.front();
            queue.pop();
        }
        signalPop.notify_all();
    }

    bool tryWaitAndPop(T& _value, int _milli)
    {
        {
            std::unique_lock<std::mutex> lock(guard);
            while (queue.empty())
            {
                signalPush.wait_for(lock, std::chrono::milliseconds(_milli));
                return false;
            }

            _value = queue.front();
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }


    void waitEmpty()
    {
        std::unique_lock<std::mutex> lock(guard);
        while (!queue.empty())
        {
            signalPop.wait(lock);
        }
    }



private:
    unsigned maxsize = std::numeric_limits<unsigned>::max();
    std::queue<T> queue;
    mutable std::mutex guard;
    std::condition_variable signalPop;
    std::condition_variable signalPush;
};
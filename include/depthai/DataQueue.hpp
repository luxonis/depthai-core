#pragma once

// std
#include <atomic>
#include <memory>

// project
#include "LockingQueue.hpp"
#include "pipeline/datatype/ADatatype.hpp"
#include "xlink/XLinkConnection.hpp"

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

// DataQueue presents a way to access data coming from MyriadX
class DataOutputQueue {
    std::shared_ptr<LockingQueue<std::shared_ptr<ADatatype>>> pQueue;
    LockingQueue<std::shared_ptr<ADatatype>>& queue;
    std::thread readingThread;
    std::shared_ptr<std::atomic<bool>> pRunning;
    std::atomic<bool>& running;
    std::shared_ptr<std::string> pExceptionMessage;
    std::string& exceptionMessage;
    std::string streamName;

    // const std::chrono::milliseconds READ_TIMEOUT{500};

   public:
    DataOutputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize = 60, bool overwrite = false);
    ~DataOutputQueue();

    std::string getName() const;

    template <class T>
    bool has() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(queue.front(val) && dynamic_cast<T*>(val.get())) {
            return true;
        }
        return false;
    }

    bool has() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        return !queue.empty();
    }

    template <class T>
    std::shared_ptr<T> tryGet() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryPop(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    std::shared_ptr<ADatatype> tryGet() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> p = nullptr;
        if(!queue.tryPop(p)) {
            return nullptr;
        }
        return p;
    }

    template <class T>
    std::shared_ptr<T> get() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.waitAndPop(val)) {
            throw std::runtime_error(exceptionMessage.c_str());
            return nullptr;
        }
        return std::dynamic_pointer_cast<T>(val);
    }

    std::shared_ptr<ADatatype> get() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.waitAndPop(val)) {
            throw std::runtime_error(exceptionMessage.c_str());
            return nullptr;
        }
        return val;
    }

    template <class T, typename Rep, typename Period>
    std::shared_ptr<T> get(std::chrono::duration<Rep, Period> timeout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryWaitAndPop(val, timeout)) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<T>(val);
    }

    template <typename Rep, typename Period>
    std::shared_ptr<ADatatype> get(std::chrono::duration<Rep, Period> timeout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryWaitAndPop(val, timeout)) {
            return nullptr;
        }
        return val;
    }
};

// DataInputQueue presents a way to write to MyriadX
class DataInputQueue {
    std::shared_ptr<LockingQueue<std::shared_ptr<RawBuffer>>> pQueue;
    LockingQueue<std::shared_ptr<RawBuffer>>& queue;
    std::thread writingThread;
    std::shared_ptr<std::atomic<bool>> pRunning;
    std::atomic<bool>& running;
    std::shared_ptr<std::string> pExceptionMessage;
    std::string& exceptionMessage;
    std::string streamName;

   public:
    DataInputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize = 60, bool overwrite = false);
    ~DataInputQueue();

    std::string getName() const;

    void send(const std::shared_ptr<RawBuffer>& val);
    void send(const std::shared_ptr<ADatatype>& val);
    void send(const ADatatype& val);

    void sendSync(const std::shared_ptr<RawBuffer>& val);
    void sendSync(const std::shared_ptr<ADatatype>& val);
    void sendSync(const ADatatype& val);

    template <typename Rep, typename Period>
    bool send(const std::shared_ptr<RawBuffer>& val, std::chrono::duration<Rep, Period> timeout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        return queue.tryWaitAndPush(val, timeout);
    }
    template <typename Rep, typename Period>
    bool send(const std::shared_ptr<ADatatype>& val, std::chrono::duration<Rep, Period> timeout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        return queue.tryWaitAndPush(val->serialize(), timeout);
    }
};

}  // namespace dai

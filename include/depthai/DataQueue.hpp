#pragma once

// std
#include <atomic>
#include <memory>

// project
#include "LockingQueue.hpp"
#include "xlink/XLinkConnection.hpp"

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

// DataQueue presents a way to access data coming from MyriadX
class DataOutputQueue {
    std::shared_ptr<XLinkConnection> connection;
    LockingQueue<std::shared_ptr<RawBuffer>> queue;
    std::thread readingThread;
    std::atomic<bool> running{true};

   public:
    DataOutputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize = 60, bool overwrite = false);
    ~DataOutputQueue();

    template <class T>
    bool has() {
        std::shared_ptr<RawBuffer> val;
        if(queue.front(val) && dynamic_cast<T*>(val.get())) {
            return true;
        }
        return false;
    }

    bool has() {
        return !queue.empty();
    }

    template <class T>
    std::shared_ptr<T> tryGet() {
        std::shared_ptr<RawBuffer> val;
        if(!queue.tryPop(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    std::shared_ptr<RawBuffer> tryGet() {
        std::shared_ptr<RawBuffer> p;
        if(!queue.tryPop(p)) {
            return nullptr;
        }
        return p;
    }

    template <class T>
    std::shared_ptr<T> get() {
        std::shared_ptr<RawBuffer> val;
        queue.waitAndPop(val);
        return std::dynamic_pointer_cast<T>(val);
    }

    std::shared_ptr<RawBuffer> get() {
        std::shared_ptr<RawBuffer> val;
        queue.waitAndPop(val);
        return val;
    }
};

// DataInputQueue presents a way to write to MyriadX
class DataInputQueue {
    std::shared_ptr<XLinkConnection> connection;
    LockingQueue<std::shared_ptr<RawBuffer>> queue;
    std::thread writingThread;
    std::atomic<bool> running{true};
    std::string name;

   public:
    DataInputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize = 60, bool overwrite = false);
    ~DataInputQueue();

    void send(const std::shared_ptr<RawBuffer>& val);
    void sendAsync(const std::shared_ptr<RawBuffer>& val);
};

}  // namespace dai

#pragma once

// std
#include <atomic>
#include <memory>
#include <vector>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

// DataQueue presents a way to access data coming from MyriadX
class DataOutputQueue {
    LockingQueue<std::shared_ptr<ADatatype>> queue;
    std::thread readingThread;
    std::atomic<bool> running{true};
    std::string exceptionMessage{""};
    const std::string name{""};
    std::mutex callbacksMtx;
    std::unordered_map<int, std::function<void(std::string, std::shared_ptr<ADatatype>)>> callbacks;
    int uniqueCallbackId{0};

    // const std::chrono::milliseconds READ_TIMEOUT{500};

   public:
    DataOutputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize = 16, bool blocking = true);
    ~DataOutputQueue();

    /**
     * Sets queue behaviour when full (maxSize)
     *
     * @param blocking Specifies if block or overwrite the oldest message in the queue
     */
    void setBlocking(bool blocking);

    /**
     * Gets current queue behaviour when full (maxSize)
     *
     * @return true if blocking, false otherwise
     */
    bool getBlocking() const;

    /**
     * Sets queue maximum size
     *
     * @param maxSize Specifies maximum number of messages in the queue
     */
    void setMaxSize(unsigned int maxSize);

    /**
     * Gets queue maximum size
     *
     * @return Maximum queue size
     */
    unsigned int getMaxSize(unsigned int maxSize) const;

    /**
     * Gets queues name
     *
     * @return Queue name
     */
    std::string getName() const;

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with queue name and message pointer
     * @return Callback id
     */
    int addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with message pointer
     * @return Callback id
     */
    int addCallback(std::function<void(std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function without any parameters
     * @return Callback id
     */
    int addCallback(std::function<void()> callback);

    /**
     * Removes a callback
     *
     * @param callbackId Id of callback to be removed
     * @return true if callback was removed, false otherwise
     */
    bool removeCallback(int callbackId);

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
        return tryGet<ADatatype>();
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
        return get<ADatatype>();
    }

    template <class T, typename Rep, typename Period>
    std::shared_ptr<T> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryWaitAndPop(val, timeout)) {
            hasTimedout = true;
            return nullptr;
        }
        hasTimedout = false;
        return std::dynamic_pointer_cast<T>(val);
    }

    template <typename Rep, typename Period>
    std::shared_ptr<ADatatype> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return get<ADatatype>(timeout, hasTimedout);
    }

    // Methods to retrieve multiple messages at once
    template <class T>
    std::vector<std::shared_ptr<T>> tryGetAll() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        queue.consumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    std::vector<std::shared_ptr<ADatatype>> tryGetAll() {
        return tryGetAll<ADatatype>();
    }

    template <class T>
    std::vector<std::shared_ptr<T>> getAll() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        queue.waitAndConsumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    std::vector<std::shared_ptr<ADatatype>> getAll() {
        return getAll<ADatatype>();
    }

    template <class T, typename Rep, typename Period>
    std::vector<std::shared_ptr<T>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        hasTimedout = !queue.waitAndConsumeAll(
            [&messages](std::shared_ptr<ADatatype>& msg) {
                // dynamic pointer cast may return nullptr
                // in which case that message in vector will be nullptr
                messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
            },
            timeout);

        return messages;
    }

    template <typename Rep, typename Period>
    std::vector<std::shared_ptr<ADatatype>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return getAll<ADatatype>(timeout, hasTimedout);
    }
};

// DataInputQueue presents a way to write to MyriadX
class DataInputQueue {
    LockingQueue<std::shared_ptr<RawBuffer>> queue;
    std::thread writingThread;
    std::atomic<bool> running;
    std::string exceptionMessage;
    const std::string name;
    std::size_t maxDataSize;

   public:
    DataInputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize = 16, bool blocking = true);
    ~DataInputQueue();

    /**
     * Sets maximum message size. If message is larger than specified, then an exception is issued.
     *
     * @param maxSize Maximum message size to add to queue
     */
    void setMaxDataSize(std::size_t maxSize);

    /**
     * Gets maximum queue size.
     *
     * @return Maximum message size
     */
    std::size_t getMaxDataSize();

    /**
     * Sets queue behaviour when full (maxSize)
     *
     * @param blocking Specifies if block or overwrite the oldest message in the queue
     */
    void setBlocking(bool blocking);

    /**
     * Gets current queue behaviour when full (maxSize)
     *
     * @return true if blocking, false otherwise
     */
    bool getBlocking() const;

    /**
     * Sets queue maximum size
     *
     * @param maxSize Specifies maximum number of messages in the queue
     */
    void setMaxSize(unsigned int maxSize);

    /**
     * Gets queue maximum size
     *
     * @return Maximum queue size
     */
    unsigned int getMaxSize(unsigned int maxSize) const;

    /**
     * Gets queues name
     *
     * @return Queue name
     */
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

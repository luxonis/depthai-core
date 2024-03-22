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
namespace dai {

/**
 * Thread safe queue to send messages between nodes
 */
class MessageQueue {
   public:
    /// Alias for callback id
    using CallbackId = int;

    class QueueException : public std::runtime_error {
       public:
        explicit QueueException(const std::string& message) : std::runtime_error(message) {}
    };

   private:
    LockingQueue<std::shared_ptr<ADatatype>> queue;
    const std::string name{""};
    std::mutex callbacksMtx;
    std::unordered_map<CallbackId, std::function<void(std::string, std::shared_ptr<ADatatype>)>> callbacks;
    CallbackId uniqueCallbackId{0};
    const std::string exceptionMessage{"MessageQueue was closed"};
    void callCallbacks(std::shared_ptr<ADatatype> msg);

   public:
    // DataOutputQueue constructor
    MessageQueue(unsigned int maxSize = 16, bool blocking = true);
    MessageQueue(const std::string& name, unsigned int maxSize = 16, bool blocking = true);
    MessageQueue(const MessageQueue& q) : name(q.name), exceptionMessage(q.exceptionMessage) {
        callbacks = q.callbacks;
        uniqueCallbackId = q.uniqueCallbackId;
    }
    MessageQueue(MessageQueue&& q) : name(std::move(q.name)), exceptionMessage(std::move(q.exceptionMessage)) {
        callbacks = std::move(q.callbacks);
        uniqueCallbackId = std::move(q.uniqueCallbackId);
    }
    ~MessageQueue();

    /**
     * Check whether queue is closed
     */
    bool isClosed() const;

    /**
     * Closes the queue and unblocks any waiting consumers or producers
     */
    void close();

    /**
     * Sets queue behavior when full (maxSize)
     *
     * @param blocking Specifies if block or overwrite the oldest message in the queue
     */
    void setBlocking(bool blocking);

    /**
     * Gets current queue behavior when full (maxSize)
     *
     * @returns True if blocking, false otherwise
     */
    bool getBlocking() const;

    /**
     * Sets queue maximum size
     *
     * @param maxSize Specifies maximum number of messages in the queue
     * @note If maxSize is smaller than size, queue will not be truncated immediately, only after messages are popped
     */
    void setMaxSize(unsigned int maxSize);

    /**
     * Gets queue maximum size
     *
     * @returns Maximum queue size
     */
    unsigned int getMaxSize() const;

    /**
     * Gets queue current size
     *
     * @returns Current queue size
     */
    unsigned int getSize() const;

    /**
     * Gets whether queue is full
     *
     * @returns True if queue is full, false otherwise
     */
    unsigned int isFull() const;

    /**
     * Gets queues name
     *
     * @returns Queue name
     */
    std::string getName() const;

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with queue name and message pointer
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with message pointer
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void(std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function without any parameters
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void()> callback);

    /**
     * Removes a callback
     *
     * @param callbackId Id of callback to be removed
     * @returns True if callback was removed, false otherwise
     */
    bool removeCallback(CallbackId callbackId);

    /**
     * Check whether front of the queue has message of type T
     * @returns True if queue isn't empty and the first element is of type T, false otherwise
     */
    template <class T>
    bool has() {
        std::shared_ptr<ADatatype> val = nullptr;
        if(queue.front(val) && dynamic_cast<T*>(val.get())) {
            return true;
        }
        return false;
    }

    /**
     * Check whether front of the queue has a message (isn't empty)
     * @returns True if queue isn't empty, false otherwise
     */
    bool has() {
        return !queue.empty();
    }

    /**
     * Try to retrieve message T from queue. If message isn't of type T it returns nullptr
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> tryGet() {
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryPop(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Try to retrieve message from queue. If no message available, return immediately with nullptr
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> tryGet() {
        return tryGet<ADatatype>();
    }

    /**
     * Block until a message is available.
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> get() {
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.waitAndPop(val)) {
            throw QueueException(exceptionMessage.c_str());
        }
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Block until a message is available.
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> get() {
        return get<ADatatype>();
    }

    /**
     * Gets first message in the queue.
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> front() {
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.front(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Gets first message in the queue.
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> front() {
        return front<ADatatype>();
    }

    /**
     * Block until a message is available with a timeout.
     *
     * @param timeout Duration for which the function should block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Message of type T otherwise nullptr if message isn't type T or timeout occurred
     */
    template <class T, typename Rep, typename Period>
    std::shared_ptr<T> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryWaitAndPop(val, timeout)) {
            hasTimedout = true;
            return nullptr;
        }
        hasTimedout = false;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Block until a message is available with a timeout.
     *
     * @param timeout Duration for which the function should block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Message of type T otherwise nullptr if message isn't type T or timeout occurred
     */
    template <typename Rep, typename Period>
    std::shared_ptr<ADatatype> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return get<ADatatype>(timeout, hasTimedout);
    }

    /**
     * Try to retrieve all messages in the queue.
     *
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T>
    std::vector<std::shared_ptr<T>> tryGetAll() {
        std::vector<std::shared_ptr<T>> messages;
        queue.consumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    /**
     * Try to retrieve all messages in the queue.
     *
     * @returns Vector of messages
     */
    std::vector<std::shared_ptr<ADatatype>> tryGetAll() {
        return tryGetAll<ADatatype>();
    }

    /**
     * Block until at least one message in the queue.
     * Then return all messages from the queue.
     *
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T>
    std::vector<std::shared_ptr<T>> getAll() {
        std::vector<std::shared_ptr<T>> messages;
        queue.waitAndConsumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    /**
     * Block until at least one message in the queue.
     * Then return all messages from the queue.
     *
     * @returns Vector of messages
     */
    std::vector<std::shared_ptr<ADatatype>> getAll() {
        return getAll<ADatatype>();
    }

    /**
     * Block for maximum timeout duration.
     * Then return all messages from the queue.
     * @param timeout Maximum duration to block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T, typename Rep, typename Period>
    std::vector<std::shared_ptr<T>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
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

    /**
     * Block for maximum timeout duration.
     * Then return all messages from the queue.
     * @param timeout Maximum duration to block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Vector of messages
     */
    template <typename Rep, typename Period>
    std::vector<std::shared_ptr<ADatatype>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return getAll<ADatatype>(timeout, hasTimedout);
    }

    /**
     * Adds a message to the queue, which will be picked up and sent to the device.
     * Can either block if 'blocking' behavior is true or overwrite oldest
     * @param msg Message to add to the queue
     */
    void send(const std::shared_ptr<ADatatype>& msg);

    /**
     * Adds a message to the queue, which will be picked up and sent to the device.
     * Can either block if 'blocking' behavior is true or overwrite oldest
     * @param msg Message to add to the queue
     */
    // void send(const ADatatype& msg);

    /**
     * Adds message to the queue, which will be picked up and sent to the device.
     * Can either block until timeout if 'blocking' behavior is true or overwrite oldest
     *
     * @param msg Message to add to the queue
     * @param timeout Maximum duration to block in milliseconds
     */
    bool send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout);

    /**
     * Adds message to the queue, which will be picked up and sent to the device.
     * Can either block until timeout if 'blocking' behavior is true or overwrite oldest
     *
     * @param msg Message to add to the queue
     * @param timeout Maximum duration to block in milliseconds
     */
    bool send(const ADatatype& msg, std::chrono::milliseconds timeout);

    /**
     * Tries sending a message
     *
     * @param msg message to send
     */
    bool trySend(const std::shared_ptr<ADatatype>& msg);
    // bool trySend(const ADatatype& msg);
};

}  // namespace dai
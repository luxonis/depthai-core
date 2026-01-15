#pragma once

// std
#include <condition_variable>
#include <memory>
#include <utility>
#include <vector>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "depthai/utility/PipelineEventDispatcherInterface.hpp"

// shared
namespace dai {

/**
 * Thread safe queue to send messages between nodes
 */
class MessageQueue : public std::enable_shared_from_this<MessageQueue> {
   public:
    /// Alias for callback id
    using CallbackId = int;

    class QueueException : public std::runtime_error {
       public:
        explicit QueueException(const std::string& message) : std::runtime_error(message) {}
        ~QueueException() noexcept override;
    };

   private:
    static constexpr auto CLOSED_QUEUE_MESSAGE = "MessageQueue was closed";
    LockingQueue<std::shared_ptr<ADatatype>> queue;
    std::string name;

    std::mutex cvNotifyMtx;
    std::unordered_map<CallbackId, std::shared_ptr<std::condition_variable>> condVars;
    CallbackId uniqueCondVarId{0};

   public:
    std::mutex callbacksMtx;                                                                                 // Only public for the Python bindings
    std::unordered_map<CallbackId, std::function<void(std::string, std::shared_ptr<ADatatype>)>> callbacks;  // Only public for the Python bindings
    CallbackId uniqueCallbackId{0};

   private:
    void callCallbacks(std::shared_ptr<ADatatype> msg);
    void notifyCondVars();
    utility::PipelineEventDispatcherInterface* pipelineEventDispatcher = nullptr;

   public:
    // DataOutputQueue constructor
    explicit MessageQueue(unsigned int maxSize = 16, bool blocking = true);
    explicit MessageQueue(std::string name,
                          unsigned int maxSize = 16,
                          bool blocking = true,
                          utility::PipelineEventDispatcherInterface* pipelineEventDispatcher = nullptr);

    MessageQueue(const MessageQueue& c)
        : enable_shared_from_this(c),
          queue(c.queue),
          name(c.name),
          callbacks(c.callbacks),
          uniqueCallbackId(c.uniqueCallbackId),
          pipelineEventDispatcher(c.pipelineEventDispatcher){};
    MessageQueue(MessageQueue&& m) noexcept
        : enable_shared_from_this(m),
          queue(std::move(m.queue)),
          name(std::move(m.name)),
          callbacks(std::move(m.callbacks)),
          uniqueCallbackId(m.uniqueCallbackId),
          pipelineEventDispatcher(m.pipelineEventDispatcher){};

    MessageQueue& operator=(const MessageQueue& c) {
        queue = c.queue;
        name = c.name;
        callbacks = c.callbacks;
        uniqueCallbackId = c.uniqueCallbackId;
        pipelineEventDispatcher = c.pipelineEventDispatcher;
        return *this;
    }

    MessageQueue& operator=(MessageQueue&& m) noexcept {
        queue = std::move(m.queue);
        name = std::move(m.name);
        callbacks = std::move(m.callbacks);
        uniqueCallbackId = m.uniqueCallbackId;
        pipelineEventDispatcher = m.pipelineEventDispatcher;
        return *this;
    }

    static std::unordered_map<std::string, std::shared_ptr<ADatatype>> getAny(std::unordered_map<std::string, MessageQueue&> queues, std::optional<std::chrono::milliseconds> timeout = std::nullopt);
    template <typename T>
    static std::unordered_map<std::string, std::shared_ptr<T>> getAny(std::unordered_map<std::string, MessageQueue&> queues, std::optional<std::chrono::milliseconds> timeout = std::nullopt) {
        auto resultADatatype = getAny(std::move(queues), timeout);
        std::unordered_map<std::string, std::shared_ptr<T>> result;
        for(auto& [k, v] : resultADatatype) {
            auto casted = std::dynamic_pointer_cast<T>(v);
            if(casted) {
                result[k] = casted;
            }
        }
        return result;
    }

    virtual ~MessageQueue();

    /**
     * @brief Get name of the queue
     */
    std::string getName() const;

    /**
     * Set the name of the queue
     */
    void setName(std::string name);

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
    CallbackId addCallback(const std::function<void(std::shared_ptr<ADatatype>)>&);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function without any parameters
     * @returns Callback id
     */
    CallbackId addCallback(const std::function<void()>& callback);

    /**
     * Adds a condition variable to be notified on message queue destruction
     *
     * @param cv Condition variable to be notified
     * @returns Condition variable id
     */
    CallbackId addCondVar(std::shared_ptr<std::condition_variable> cv);

    /**
     * Removes a callback
     *
     * @param callbackId Id of callback to be removed
     * @returns True if callback was removed, false otherwise
     */
    bool removeCallback(CallbackId callbackId);

    /**
     * Removes a condition variable
     *
     * @param condVarId Id of condition variable to be removed
     * @returns True if condition variable was removed, false otherwise
     */
    bool removeCondVar(CallbackId condVarId);

    /**
     * Check whether front of the queue has message of type T
     * @returns True if queue isn't empty and the first element is of type T, false otherwise
     */
    template <class T>
    bool has() {
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        std::shared_ptr<ADatatype> val = nullptr;
        return queue.front(val) && dynamic_cast<T*>(val.get());
    }

    /**
     * Check whether front of the queue has a message (isn't empty)
     * @returns True if queue isn't empty, false otherwise
     */
    bool has() {
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        return !queue.empty();
    }

    /**
     * Try to retrieve message T from queue. If message isn't of type T it returns nullptr
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> tryGet() {
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        auto getInput = [this]() -> std::shared_ptr<T> {
            std::shared_ptr<ADatatype> val = nullptr;
            if(!this->queue.tryPop(val)) {
                return nullptr;
            }
            return std::dynamic_pointer_cast<T>(val);
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            auto result = getInput();
            blockEvent.setQueueSize(getSize());
            if(!result) {
                blockEvent.cancel();
            }
            return result;
        } else {
            return getInput();
        }
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
        auto getInput = [this, &val]() {
            if(!this->queue.waitAndPop(val)) {
                throw QueueException(CLOSED_QUEUE_MESSAGE);
            }
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            getInput();
            blockEvent.setQueueSize(getSize());
        } else {
            getInput();
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
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
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
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        auto getInput = [&, this]() -> std::shared_ptr<T> {
            std::shared_ptr<ADatatype> val = nullptr;
            if(!this->queue.tryWaitAndPop(val, timeout)) {
                hasTimedout = true;
                // Check again after the timeout
                if(this->queue.isDestroyed()) {
                    throw QueueException(CLOSED_QUEUE_MESSAGE);
                }
                return nullptr;
            }
            hasTimedout = false;
            return std::dynamic_pointer_cast<T>(val);
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            auto result = getInput();
            blockEvent.setQueueSize(getSize());
            if(!result || hasTimedout) {
                blockEvent.cancel();
            }
            return result;
        } else {
            return getInput();
        }
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
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        auto getInput = [this]() -> std::vector<std::shared_ptr<T>> {
            std::vector<std::shared_ptr<T>> messages;
            this->queue.consumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
                // dynamic pointer cast may return nullptr
                // in which case that message in vector will be nullptr
                messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
            });
            return messages;
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            auto result = getInput();
            blockEvent.setQueueSize(getSize());
            if(result.empty()) {
                blockEvent.cancel();
            }
            return result;
        } else {
            return getInput();
        }
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
        auto getInput = [this, &messages]() {
            bool notDestructed = this->queue.waitAndConsumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
                // dynamic pointer cast may return nullptr
                // in which case that message in vector will be nullptr
                messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
            });
            if(!notDestructed) {
                throw QueueException(CLOSED_QUEUE_MESSAGE);
            }
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            getInput();
            blockEvent.setQueueSize(getSize());
        } else {
            getInput();
        }
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
        if(queue.isDestroyed()) {
            throw QueueException(CLOSED_QUEUE_MESSAGE);
        }
        auto getInput = [&, this]() -> std::vector<std::shared_ptr<T>> {
            std::vector<std::shared_ptr<T>> messages;
            hasTimedout = !this->queue.waitAndConsumeAll(
                [&messages](std::shared_ptr<ADatatype>& msg) {
                    // dynamic pointer cast may return nullptr
                    // in which case that message in vector will be nullptr
                    messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
                },
                timeout);

            return messages;
        };
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::INPUT, name);
            auto result = getInput();
            blockEvent.setQueueSize(getSize());
            if(result.empty() || hasTimedout) {
                blockEvent.cancel();
            }
            return result;
        } else {
            return getInput();
        }
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
};

}  // namespace dai

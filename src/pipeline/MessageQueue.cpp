#include "depthai/pipeline/MessageQueue.hpp"

// std
#include <chrono>
#include <iostream>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"

// libraries
#include "spdlog/spdlog.h"

// Additions
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "utility/PipelineEventDispatcherInterface.hpp"

namespace dai {

MessageQueue::MessageQueue(std::string name, unsigned int maxSize, bool blocking, utility::PipelineEventDispatcherInterface* pipelineEventDispatcher)
    : queue(maxSize, blocking), name(std::move(name)), pipelineEventDispatcher(pipelineEventDispatcher) {}

MessageQueue::MessageQueue(unsigned int maxSize, bool blocking) : queue(maxSize, blocking) {}

bool MessageQueue::isClosed() const {
    return queue.isDestroyed();
}

void MessageQueue::close() {
    // Destroy queue
    queue.destruct();

    notifyCondVars();

    // Log if name not empty
    if(!name.empty()) spdlog::debug("MessageQueue ({}) closed", name);
}

MessageQueue::~MessageQueue() {
    // Close the queue first
    close();
}

void MessageQueue::setName(std::string name) {
    this->name = std::move(name);
}

std::string MessageQueue::getName() const {
    return name;
}

void MessageQueue::setBlocking(bool blocking) {
    queue.setBlocking(blocking);
}

bool MessageQueue::getBlocking() const {
    return queue.getBlocking();
}

void MessageQueue::setMaxSize(unsigned int maxSize) {
    queue.setMaxSize(maxSize);
}

unsigned int MessageQueue::getMaxSize() const {
    return queue.getMaxSize();
}

unsigned int MessageQueue::getSize() const {
    return queue.getSize();
}

unsigned int MessageQueue::isFull() const {
    return queue.isFull();
}

int MessageQueue::addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)> callback) {
    // Lock first
    std::unique_lock<std::mutex> lock(callbacksMtx);

    // Get unique id
    int uniqueId = uniqueCallbackId++;

    // assign callback
    callbacks[uniqueId] = std::move(callback);

    // return id assigned to the callback
    return uniqueId;
}

int MessageQueue::addCallback(const std::function<void(std::shared_ptr<ADatatype>)>& callback) {
    // Create a wrapper
    return addCallback([callback](const std::string&, std::shared_ptr<ADatatype> message) { callback(std::move(message)); });
}

int MessageQueue::addCallback(const std::function<void()>& callback) {
    // Create a wrapper
    return addCallback([callback](const std::string&, std::shared_ptr<ADatatype>) { callback(); });
}

int MessageQueue::addCondVar(std::shared_ptr<std::condition_variable> cv) {
    // Lock first
    std::unique_lock<std::mutex> lock(cvNotifyMtx);

    // Get unique id
    int uniqueId = uniqueCondVarId++;

    // assign callback
    condVars[uniqueId] = std::move(cv);

    // return id assigned to the callback
    return uniqueId;
}

bool MessageQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> lock(callbacksMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

bool MessageQueue::removeCondVar(int condVarId) {
    // Lock first
    std::unique_lock<std::mutex> lock(cvNotifyMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(condVars.count(condVarId) == 0) return false;

    // Otherwise erase and return true
    condVars.erase(condVarId);
    return true;
}

void MessageQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    callCallbacks(msg);
    auto queueNotClosed = queue.push(msg, [&](LockingQueueState state, size_t size) {
        if(pipelineEventDispatcher) {
            switch(state) {
                case LockingQueueState::BLOCKED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::BLOCKED, size);
                    break;
                case LockingQueueState::CANCELLED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::CANCELLED, size);
                    break;
                case LockingQueueState::OK:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::SUCCESS, size);
                    break;
            }
        }
    });
    if(!queueNotClosed) throw QueueException(CLOSED_QUEUE_MESSAGE);
}

bool MessageQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    return queue.tryWaitAndPush(msg, timeout, [&](LockingQueueState state, size_t size) {
        if(pipelineEventDispatcher) {
            switch(state) {
                case LockingQueueState::BLOCKED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::BLOCKED, size);
                    break;
                case LockingQueueState::CANCELLED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::CANCELLED, size);
                    break;
                case LockingQueueState::OK:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::SUCCESS, size);
                    break;
            }
        }
    });
}

bool MessageQueue::trySend(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    return send(msg, std::chrono::milliseconds(0));
}

void MessageQueue::callCallbacks(std::shared_ptr<ADatatype> message) {
    // Lock first
    std::lock_guard<std::mutex> lock(callbacksMtx);

    // Call all callbacks
    for(auto& keyValue : callbacks) {
        keyValue.second(name, message);
    }
}

void MessageQueue::notifyCondVars() {
    // Lock first
    std::lock_guard<std::mutex> lock(cvNotifyMtx);

    // Call all callbacks
    for(auto& keyValue : condVars) {
        keyValue.second->notify_one();
    }
}

MessageQueue::QueueException::~QueueException() noexcept = default;

std::unordered_map<std::string, std::shared_ptr<ADatatype>> getAny(std::unordered_map<std::string, MessageQueue&> queues) {
    std::vector<std::pair<MessageQueue&, MessageQueue::CallbackId>> callbackIds;
    std::vector<std::pair<MessageQueue&, MessageQueue::CallbackId>> condVarIds;
    callbackIds.reserve(queues.size());
    condVarIds.reserve(queues.size());
    std::unordered_map<std::string, std::shared_ptr<ADatatype>> inputs;

    std::mutex inputsWaitMutex;
    auto inputsWaitCv = std::make_shared<std::condition_variable>();
    bool receivedMessage = false;

    // Register callbacks
    for(auto& kv : queues) {
        auto& input = kv.second;
        auto callbackId = input.addCallback([&]() {
            {
                std::lock_guard<std::mutex> lock(inputsWaitMutex);
                receivedMessage = true;
            }
            inputsWaitCv->notify_all();
        });

        callbackIds.push_back({input, callbackId});
        // Also add condition variable to be notified on close
        auto condVarId = input.addCondVar(inputsWaitCv);
        condVarIds.push_back({input, condVarId});
    }

    // Check if any messages already present
    bool hasAnyMessages = false;
    for(auto& kv : queues) {
        if(kv.second.has()) {
            hasAnyMessages = true;
            break;
        }
    }

    if(!hasAnyMessages) {
        // Wait for any message to arrive
        std::unique_lock<std::mutex> lock(inputsWaitMutex);
        inputsWaitCv->wait(lock, [&]() {
            for(auto& kv : queues) {
                if(kv.second.isClosed()) {
                    return true;
                }
            }
            return receivedMessage;
        });
    }

    // Remove callbacks
    for(auto& [input, callbackId] : callbackIds) {
        input.removeCallback(callbackId);
    }
    // Remove condition variables
    for(auto& [input, condVarId] : condVarIds) {
        input.removeCondVar(condVarId);
    }

    // Collect all available messages
    for(auto& kv : queues) {
        auto& input = kv.second;
        if(input.has()) {
            inputs[kv.first] = input.get<ADatatype>();
        }
    }

    return inputs;
}

}  // namespace dai

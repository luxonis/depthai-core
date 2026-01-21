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

    notifyListeners();

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

int MessageQueue::addNotifier(std::shared_ptr<utility::ManyToOneNotifier> notifier) {
    // Lock first
    std::unique_lock<std::mutex> lock(notifierMtx);

    // Get unique id
    int uniqueId = uniqueNotifierId++;

    // assign callback
    notifiers[uniqueId] = std::move(notifier);

    // return id assigned to the callback
    return uniqueId;
}

bool MessageQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> lock(callbacksMtx);

    // If callback with id 'callbackId' doesn't exist, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

bool MessageQueue::removeNotifier(int notifierId) {
    // Lock first
    std::unique_lock<std::mutex> lock(notifierMtx);

    // If callback with id 'notifierId' doesn't exist, return false
    if(notifiers.count(notifierId) == 0) return false;

    // Otherwise erase and return true
    notifiers.erase(notifierId);
    return true;
}

void MessageQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    callCallbacks(msg);
    auto queueNotClosed = queue.push(msg, [&](LockingQueueState state, size_t size) {
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            switch(state) {
                case LockingQueueState::BLOCKED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::BLOCKED, size);
                    break;
                case LockingQueueState::CANCELLED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::CANCELLED, size);
                    break;
                case LockingQueueState::SUCCESS:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::SUCCESS, size);
                    break;
            }
        }
    });
    notifyListeners();
    if(!queueNotClosed) throw QueueException(CLOSED_QUEUE_MESSAGE);
}

bool MessageQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    auto ret = queue.tryWaitAndPush(msg, timeout, [&](LockingQueueState state, size_t size) {
        if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
            switch(state) {
                case LockingQueueState::BLOCKED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::BLOCKED, size);
                    break;
                case LockingQueueState::CANCELLED:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::CANCELLED, size);
                    break;
                case LockingQueueState::SUCCESS:
                    pipelineEventDispatcher->pingInputEvent(name, PipelineEvent::Status::SUCCESS, size);
                    break;
            }
        }
    });
    if(ret) notifyListeners();
    return ret;
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

void MessageQueue::notifyListeners() {
    // Lock first
    std::lock_guard<std::mutex> lock(notifierMtx);

    // Notify all listeners
    for(auto& keyValue : notifiers) {
        keyValue.second->notifyOne();
    }
}

MessageQueue::QueueException::~QueueException() noexcept = default;

bool MessageQueue::waitAny(const std::vector<MessageQueue*>& queues, std::optional<std::chrono::milliseconds> timeout) {
    std::vector<std::pair<MessageQueue*, MessageQueue::CallbackId>> notifierIds;
    notifierIds.reserve(queues.size());

    auto removeNotifiers = [&]() {
        for(auto& [input, notifierId] : notifierIds) {
            input->removeNotifier(notifierId);
        }
    };
    auto checkForMessages = [&]() {
        for(const auto& q : queues) {
            if(q->has()) {
                return true;
            }
        }
        return false;
    };
    auto pred = [&]() {
        for(const auto& q : queues) {
            if(q->isClosed()) {
                return true;
            }
        }
        return checkForMessages();
    };

    std::shared_ptr<utility::ManyToOneNotifier> notifier = std::make_shared<utility::ManyToOneNotifier>();
    try {
        // Add notifier to all queues, if any message arrives from this point, wait should return
        for(auto* input : queues) {
            auto notifierId = input->addNotifier(notifier);
            notifierIds.emplace_back(input, notifierId);
        }
        // Check if any messages already present
        if(!checkForMessages()) {
            if(timeout.has_value()) {
                return notifier->waitFor(pred, timeout.value());
            }
            notifier->wait(pred);
        }
    } catch(...) {
        removeNotifiers();
        throw;
    }
    removeNotifiers();
    return true;
}
std::unordered_map<std::string, std::shared_ptr<ADatatype>> MessageQueue::getAny(std::unordered_map<std::string, MessageQueue&> queues,
                                                                                 std::optional<std::chrono::milliseconds> timeout) {
    std::unordered_map<std::string, std::shared_ptr<ADatatype>> inputs;
    std::vector<MessageQueue*> queuesVec;
    queuesVec.reserve(queues.size());
    for(auto& kv : queues) {
        queuesVec.push_back(&kv.second);
    }
    bool gotAny = waitAny(queuesVec, timeout);
    if(gotAny) {
        for(auto& kv : queues) {
            auto& input = kv.second;
            if(input.has()) inputs[kv.first] = input.get<ADatatype>();
        }
    }
    return inputs;
}

}  // namespace dai

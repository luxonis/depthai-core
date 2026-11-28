#include "depthai/pipeline/MessageQueue.hpp"

#include <chrono>
#include <functional>
#include <iostream>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"
#include "utility/PipelineEventDispatcherInterface.hpp"

namespace dai {

MessageQueue::MessageQueue(std::string name, unsigned int maxSize, bool blocking, utility::PipelineEventDispatcherInterface* pipelineEventDispatcher)
    : queue(maxSize, blocking), name(std::move(name)), pipelineEventDispatcher(pipelineEventDispatcher) {}

MessageQueue::MessageQueue(unsigned int maxSize, bool blocking) : queue(maxSize, blocking) {}

bool MessageQueue::isClosed() const {
    return queue.isDestroyed();
}

void MessageQueue::close() {
    queue.destruct();

    notifyListeners();

    if(!name.empty()) spdlog::debug("MessageQueue ({}) closed", name);
}

MessageQueue::~MessageQueue() {
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
    std::unique_lock<std::mutex> lock(callbacksMtx);

    int uniqueId = uniqueCallbackId++;

    callbacks[uniqueId] = std::move(callback);

    return uniqueId;
}

int MessageQueue::addCallback(const std::function<void(std::shared_ptr<ADatatype>)>& callback) {
    return addCallback([callback](const std::string&, std::shared_ptr<ADatatype> message) { callback(std::move(message)); });
}

int MessageQueue::addCallback(const std::function<void()>& callback) {
    return addCallback([callback](const std::string&, std::shared_ptr<ADatatype>) { callback(); });
}

int MessageQueue::addNotifier(std::shared_ptr<utility::WaitAnyNotifier> notifier) {
    std::unique_lock<std::mutex> lock(notifierMtx);

    int uniqueId = uniqueNotifierId++;

    notifiers[uniqueId] = std::move(notifier);

    return uniqueId;
}

bool MessageQueue::removeCallback(int callbackId) {
    std::unique_lock<std::mutex> lock(callbacksMtx);

    if(callbacks.count(callbackId) == 0) return false;

    callbacks.erase(callbackId);
    return true;
}

bool MessageQueue::removeNotifier(int notifierId) {
    std::unique_lock<std::mutex> lock(notifierMtx);

    if(notifiers.count(notifierId) == 0) return false;

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

void MessageQueue::callCallbacks(const std::shared_ptr<ADatatype>& message) {
    std::lock_guard<std::mutex> lock(callbacksMtx);

    for(auto& keyValue : callbacks) {
        keyValue.second(name, message);
    }
}

void MessageQueue::notifyListeners() {
    std::lock_guard<std::mutex> lock(notifierMtx);

    for(auto& keyValue : notifiers) {
        keyValue.second->notifyOne();
    }
}

MessageQueue::QueueException::~QueueException() noexcept = default;

bool MessageQueue::waitAny(const std::vector<std::reference_wrapper<MessageQueue>>& queues, std::optional<std::chrono::milliseconds> timeout) {
    std::vector<std::pair<std::reference_wrapper<MessageQueue>, MessageQueue::CallbackId>> notifierIds;
    notifierIds.reserve(queues.size());

    auto removeNotifiers = [&]() {
        for(auto& [input, notifierId] : notifierIds) {
            input.get().removeNotifier(notifierId);
        }
    };
    auto checkAllClosed = [&]() {
        return std::all_of(queues.begin(), queues.end(), [](const std::reference_wrapper<MessageQueue> q) { return q.get().isClosed(); });
    };
    auto checkForMessages = [&]() {
        return std::any_of(queues.begin(), queues.end(), [](const std::reference_wrapper<MessageQueue> q) { return !q.get().isClosed() && q.get().has(); });
    };
    auto pred = [&]() {
        try {
            return checkAllClosed() || checkForMessages();
        } catch(QueueException&) {
            return true;
        }
    };

    std::shared_ptr<utility::WaitAnyNotifier> notifier = std::make_shared<utility::WaitAnyNotifier>();
    bool gotMessage = true;
    try {
        for(auto input : queues) {  // Register first, so that a message that arrives after this point ends the wait
            auto notifierId = input.get().addNotifier(notifier);
            notifierIds.emplace_back(input, notifierId);
        }
        if(!checkAllClosed() && !checkForMessages()) {
            if(timeout.has_value()) {
                notifier->waitFor(pred, timeout.value());
            } else {
                notifier->wait(pred);
            }
        }
        gotMessage = checkForMessages();
    } catch(...) {
        removeNotifiers();
        throw;
    }
    removeNotifiers();
    return gotMessage;
}
std::unordered_map<std::string, std::shared_ptr<ADatatype>> MessageQueue::getAny(const std::unordered_map<std::string, MessageQueue&>& queues,
                                                                                 std::optional<std::chrono::milliseconds> timeout) {
    std::unordered_map<std::string, std::shared_ptr<ADatatype>> inputs;
    std::vector<std::reference_wrapper<MessageQueue>> queuesVec;
    queuesVec.reserve(queues.size());
    for(const auto& kv : queues) {
        queuesVec.emplace_back(kv.second);
    }
    bool gotAny = waitAny(queuesVec, timeout);
    if(gotAny) {
        for(const auto& kv : queues) {
            auto& input = kv.second;
            std::shared_ptr<ADatatype> msg = nullptr;
            msg = input.tryGet<ADatatype>();
            if(msg) inputs[kv.first] = msg;
        }
    }
    return inputs;
}

}  // namespace dai

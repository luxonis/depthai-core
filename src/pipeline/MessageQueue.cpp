#include "depthai/pipeline/MessageQueue.hpp"

// std
#include <chrono>
#include <deque>
#include <iostream>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"

// libraries
#include "spdlog/spdlog.h"

// Additions
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

namespace dai {

MessageQueue::MessageQueue(std::string name, unsigned int maxSize, bool blocking) : queue(maxSize, blocking), name(std::move(name)) {}

MessageQueue::MessageQueue(unsigned int maxSize, bool blocking) : queue(maxSize, blocking) {}

bool MessageQueue::isClosed() const {
    return queue.isDestroyed();
}

void MessageQueue::close() {
    // Destroy queue
    queue.destruct();

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

float MessageQueue::getFps() {
    std::unique_lock<std::mutex> lock(callbacksMtx);

    // Get current time
    auto now = std::chrono::steady_clock::now();
    auto threshold = now - std::chrono::seconds(2);

    // Remove timestamps older than 2 seconds
    while (!fpsQueue.empty() && fpsQueue.front() < threshold) {
        fpsQueue.pop_front();
    }

    // If fewer than 2 timestamps are in queue, not enough data to compute FPS
    if(fpsQueue.size() < 2) {
        return 0.0;
    }

    auto oldest = fpsQueue.front();
    auto newest = fpsQueue.back();
    auto diff = std::chrono::duration<float>(newest - oldest).count();  // seconds

    // If diff is extremely small, avoid dividing by zero
    if(diff <= 0.0) {
        return 0.0;
    }
    // Using (N - 1) frames over 'diff' seconds
    // or (N) messages over 'diff' seconds—both approaches are common.
    // This calculates how many frames we got over that time window.
    return (fpsQueue.size() - 1) / diff;
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

bool MessageQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> lock(callbacksMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

void MessageQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    auto queueNotClosed = queue.push(msg);
    if(!queueNotClosed) throw QueueException(CLOSED_QUEUE_MESSAGE);

    // Record the timestamp for FPS calculation
    {
        auto now = std::chrono::steady_clock::now();
        fpsQueue.push_back(now);
        if(fpsQueue.size() > FPS_QUEUE_MAX_SIZE) {
            fpsQueue.pop_front();
        }
    }
}

bool MessageQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    if(queue.isDestroyed()) {
        throw QueueException(CLOSED_QUEUE_MESSAGE);
    }
    {
        auto now = std::chrono::steady_clock::now();
        fpsQueue.push_back(now);
        if(fpsQueue.size() > FPS_QUEUE_MAX_SIZE) {
            fpsQueue.pop_front();
        }
    }
    return queue.tryWaitAndPush(msg, timeout);
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

}  // namespace dai

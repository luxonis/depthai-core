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

namespace dai {

MessageQueue::MessageQueue(const std::string& name, unsigned int maxSize, bool blocking) : queue(maxSize, blocking), name(name) {}

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

std::string MessageQueue::getName() const {
    return name;
}

int MessageQueue::addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)> callback) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // Get unique id
    int id = uniqueCallbackId++;

    // assign callback
    callbacks[id] = callback;

    // return id assigned to the callback
    return id;
}

int MessageQueue::addCallback(std::function<void(std::shared_ptr<ADatatype>)> callback) {
    // Create a wrapper
    return addCallback([callback](std::string, std::shared_ptr<ADatatype> message) { callback(message); });
}

int MessageQueue::addCallback(std::function<void()> callback) {
    // Create a wrapper
    return addCallback([callback](std::string, std::shared_ptr<ADatatype>) { callback(); });
}

bool MessageQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

void MessageQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    queue.push(msg);
}

// void MessageQueue::send(const ADatatype& msg) {
//     send(std::make_shared<ADatatype>(msg.serialize()));
// }

bool MessageQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    return queue.tryWaitAndPush(msg, timeout);
}

// bool MessageQueue::send(const ADatatype& msg, std::chrono::milliseconds timeout) {
//     return send(std::make_shared<ADatatype>(msg.serialize()), timeout);
// }

// Try variants
bool MessageQueue::trySend(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    return send(msg, std::chrono::milliseconds(0));
}

// bool MessageQueue::trySend(const ADatatype& msg) {
//     return trySend(std::make_shared<ADatatype>(msg.serialize()));
// }

void MessageQueue::callCallbacks(std::shared_ptr<ADatatype> msg) {
    // Lock first
    std::lock_guard<std::mutex> l(callbacksMtx);

    // Call all callbacks
    for(auto& kv : callbacks) {
        kv.second(name, msg);
    }
}
}  // namespace dai

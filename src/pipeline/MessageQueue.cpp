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
    queue.push(msg);
}

bool MessageQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    callCallbacks(msg);
    return queue.tryWaitAndPush(msg, timeout);
}

bool MessageQueue::trySend(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    return send(msg, std::chrono::milliseconds(0));
}

void MessageQueue::callCallbacks(std::shared_ptr<ADatatype> message) {
    // Lock first
    std::lock_guard<std::mutex> lock(callbacksMtx);

    // Call all callbacks
    for(auto& keyValue : callbacks) {
        keyValue.second(name, std::move(message));
    }
}

}  // namespace dai

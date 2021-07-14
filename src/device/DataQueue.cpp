
#include "depthai/device/DataQueue.hpp"
// std
#include <iostream>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "pipeline/datatype/StreamPacketParser.hpp"

// shared
#include "depthai-shared/xlink/XLinkConstants.hpp"

// libraries
#include "spdlog/spdlog.h"

namespace dai {

// DATA OUTPUT QUEUE
DataOutputQueue::DataOutputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize, bool blocking)
    : queue(maxSize, blocking), name(streamName) {
    // Create stream first and then pass to thread
    // Open stream with 1B write size (no writing will happen here)
    XLinkStream stream(*conn, name, 1);

    // Creates a thread which reads from connection into the queue
    readingThread = std::thread([this, stream = std::move(stream)]() mutable {
        std::uint64_t numPacketsRead = 0;
        try {
            while(running) {
                // read packet
                streamPacketDesc_t* packet;

                // Blocking
                packet = stream.readRaw();

                // parse packet
                auto data = parsePacketToADatatype(packet);

                // Trace level debugging
                if(spdlog::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->getRaw()->serialize(metadata, type);
                    std::string objData = "/";
                    if(!metadata.empty()) objData = nlohmann::json::from_msgpack(metadata).dump();
                    spdlog::trace(
                        "Received message from device ({}) - data size: {}, object type: {} object data: {}", name, data->getRaw()->data.size(), type, objData);
                }

                // release packet
                stream.readRawRelease();

                // Add 'data' to queue
                queue.push(data);

                // Increment numPacketsRead
                numPacketsRead++;

                // Call callbacks
                {
                    std::unique_lock<std::mutex> l(callbacksMtx);
                    for(const auto& kv : callbacks) {
                        const auto& callback = kv.second;
                        try {
                            callback(name, data);
                        } catch(const std::exception& ex) {
                            spdlog::error("Callback with id: {} throwed an exception: {}", kv.first, ex.what());
                        }
                    }
                }
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        running = false;
        queue.destruct();
    });
}

bool DataOutputQueue::isClosed() const {
    return !running;
}

void DataOutputQueue::close() {
    // Set reading thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if(readingThread.joinable()) readingThread.join();

    // Log
    spdlog::debug("DataOutputQueue ({}) closed", name);
}

DataOutputQueue::~DataOutputQueue() {
    // Close the queue first
    close();

    // Then join thread
    if(readingThread.joinable()) readingThread.join();
}

void DataOutputQueue::setBlocking(bool blocking) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setBlocking(blocking);
}

bool DataOutputQueue::getBlocking() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getBlocking();
}

void DataOutputQueue::setMaxSize(unsigned int maxSize) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setMaxSize(maxSize);
}

unsigned int DataOutputQueue::getMaxSize() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getMaxSize();
}

std::string DataOutputQueue::getName() const {
    return name;
}

int DataOutputQueue::addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)> callback) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // Get unique id
    int id = uniqueCallbackId++;

    // assign callback
    callbacks[id] = callback;

    // return id assigned to the callback
    return id;
}

int DataOutputQueue::addCallback(std::function<void(std::shared_ptr<ADatatype>)> callback) {
    // Create a wrapper
    return addCallback([callback](std::string, std::shared_ptr<ADatatype> message) { callback(message); });
}

int DataOutputQueue::addCallback(std::function<void()> callback) {
    // Create a wrapper
    return addCallback([callback](std::string, std::shared_ptr<ADatatype>) { callback(); });
}

bool DataOutputQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

// DATA INPUT QUEUE
DataInputQueue::DataInputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize, bool blocking)
    : queue(maxSize, blocking), name(streamName) {
    // open stream with default XLINK_USB_BUFFER_MAX_SIZE write size
    XLinkStream stream(*conn, name, dai::XLINK_USB_BUFFER_MAX_SIZE);

    writingThread = std::thread([this, stream = std::move(stream)]() mutable {
        std::uint64_t numPacketsSent = 0;
        try {
            while(running) {
                // get data from queue
                std::shared_ptr<RawBuffer> data;
                if(!queue.waitAndPop(data)) {
                    continue;
                }

                // Trace level debugging
                if(spdlog::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->serialize(metadata, type);
                    std::string objData = "/";
                    if(!metadata.empty()) objData = nlohmann::json::from_msgpack(metadata).dump();
                    spdlog::trace("Sending message to device ({}) - data size: {}, object type: {} object data: {}", name, data->data.size(), type, objData);
                }

                // serialize
                auto serialized = serializeData(data);

                // Blocking
                stream.write(serialized);

                // Increment num packets sent
                numPacketsSent++;
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        running = false;
        queue.destruct();
    });
}

bool DataInputQueue::isClosed() const {
    return !running;
}

void DataInputQueue::close() {
    // Set reading thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if(writingThread.joinable()) writingThread.join();

    // Log
    spdlog::debug("DataInputQueue ({}) closed", name);
}

DataInputQueue::~DataInputQueue() {
    // Close the queue
    close();

    // Then join thread
    if(writingThread.joinable()) writingThread.join();
}

void DataInputQueue::setBlocking(bool blocking) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setBlocking(blocking);
}

bool DataInputQueue::getBlocking() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getBlocking();
}

void DataInputQueue::setMaxSize(unsigned int maxSize) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setMaxSize(maxSize);
}

unsigned int DataInputQueue::getMaxSize() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getMaxSize();
}

void DataInputQueue::setMaxDataSize(std::size_t maxSize) {
    maxDataSize = maxSize;
}

std::size_t DataInputQueue::getMaxDataSize() {
    return maxDataSize;
}

std::string DataInputQueue::getName() const {
    return name;
}

void DataInputQueue::send(const std::shared_ptr<RawBuffer>& rawMsg) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!rawMsg) throw std::invalid_argument("Message passed is not valid (nullptr)");

    // Check if stream receiver has enough space for this message
    if(rawMsg->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", rawMsg->data.size(), maxDataSize));
    }

    queue.push(rawMsg);
}
void DataInputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    send(msg->serialize());
}

void DataInputQueue::send(const ADatatype& msg) {
    send(msg.serialize());
}

bool DataInputQueue::send(const std::shared_ptr<RawBuffer>& rawMsg, std::chrono::milliseconds timeout) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!rawMsg) throw std::invalid_argument("Message passed is not valid (nullptr)");

    // Check if stream receiver has enough space for this message
    if(rawMsg->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", rawMsg->data.size(), maxDataSize));
    }

    return queue.tryWaitAndPush(rawMsg, timeout);
}

bool DataInputQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    return send(msg->serialize(), timeout);
}

bool DataInputQueue::send(const ADatatype& msg, std::chrono::milliseconds timeout) {
    return send(msg.serialize(), timeout);
}

}  // namespace dai

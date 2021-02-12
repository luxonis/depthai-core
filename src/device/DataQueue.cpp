
#include "depthai/device/DataQueue.hpp"
// std
#include <iostream>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/StreamPacketParser.hpp"

// shared
#include "depthai-shared/xlink/XLinkConstants.hpp"

// libraries
#include "spdlog/spdlog.h"
namespace dai {

// DATA OUTPUT QUEUE
DataOutputQueue::DataOutputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& streamName, unsigned int maxSize, bool blocking)
    : queue(maxSize, blocking), name(streamName) {
    // Creates a thread which reads from connection into the queue
    readingThread = std::thread([this, conn]() {
        constexpr auto READ_TIMEOUT = std::chrono::milliseconds(100);
        std::uint64_t numPacketsRead = 0;
        try {
            // open stream with 1B write size (no writing will happen here)
            conn->openStream(name, 1);

            while(running) {
                // read packet
                streamPacketDesc_t* packet;

                bool success;
                do {
                    success = conn->readFromStreamRaw(packet, name, READ_TIMEOUT);
                } while(!success && running);
                if(!running) break;

                // parse packet
                auto data = parsePacketToADatatype(packet);

                // Trace level debugging
                if(spdlog::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->getRaw()->serialize(metadata, type);
                    std::string objData = "/";
                    if(!metadata.empty()) objData = nlohmann::json::from_msgpack(metadata).dump();
                    spdlog::trace("Received message from device - data size: {}, object type: {} object data: {}", data->getRaw()->data.size(), type, objData);
                }

                // release packet
                conn->readFromStreamRawRelease(name);

                // Add 'data' to queue
                queue.push(data);

                // Increment numPacketsRead
                numPacketsRead++;

                // Call callbacks
                {
                    std::unique_lock<std::mutex> l(callbacksMtx);
                    for(const auto& kv : callbacks) {
                        const auto& callback = kv.second;
                        callback(name, data);
                    }
                }
            }

            conn->closeStream(name);

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        queue.destruct();
        running = false;
    });
}

DataOutputQueue::~DataOutputQueue() {
    spdlog::debug("DataOutputQueue ({}) about to be desctructed...", name);
    // Set reading thread to stop
    running = false;

    // Destroy queue
    queue.destruct();

    // join thread
    if(readingThread.joinable()) readingThread.join();
    spdlog::debug("DataOutputQueue ({}) destructed", name);
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

unsigned int DataOutputQueue::getMaxSize(unsigned int maxSize) const {
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
    writingThread = std::thread([this, conn]() {
        std::uint64_t numPacketsSent = 0;

        constexpr auto WRITE_TIMEOUT = std::chrono::milliseconds(100);
        try {
            // open stream with default XLINK_USB_BUFFER_MAX_SIZE write size
            conn->openStream(name, dai::XLINK_USB_BUFFER_MAX_SIZE);

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
                    spdlog::trace("Sending message to device - data size: {}, object type: {} object data: {}", data->data.size(), type, objData);
                }

                // serialize
                auto serialized = serializeData(data);

                bool success;
                do {
                    success = conn->writeToStream(name, serialized, WRITE_TIMEOUT);
                } while(!success && running);
                if(!running) break;

                // Increment num packets sent
                numPacketsSent++;
            }

            conn->closeStream(name);

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        queue.destruct();
        running = false;
    });
}

DataInputQueue::~DataInputQueue() {
    spdlog::debug("DataInputQueue ({}) about to be desctructed...", name);
    // Set writing thread to stop
    running = false;

    // Destroy queue
    queue.destruct();

    if(writingThread.joinable()) writingThread.join();
    spdlog::debug("DataInputQueue ({}) destructed", name);
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

unsigned int DataInputQueue::getMaxSize(unsigned int maxSize) const {
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

void DataInputQueue::send(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());

    // Check if stream receiver has enough space for this message
    if(val->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", val->data.size(), maxDataSize));
    }

    queue.push(val);
}
void DataInputQueue::send(const std::shared_ptr<ADatatype>& val) {
    send(val->serialize());
}

void DataInputQueue::send(const ADatatype& val) {
    send(val.serialize());
}

void DataInputQueue::sendSync(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());

    // Check if stream receiver has enough space for this message
    if(val->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", val->data.size(), maxDataSize));
    }

    queue.waitEmpty();
    queue.push(val);
}
void DataInputQueue::sendSync(const std::shared_ptr<ADatatype>& val) {
    sendSync(val->serialize());
}
void DataInputQueue::sendSync(const ADatatype& val) {
    sendSync(val.serialize());
}

}  // namespace dai

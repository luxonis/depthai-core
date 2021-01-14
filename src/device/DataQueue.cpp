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
DataOutputQueue::DataOutputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& name, unsigned int maxSize, bool blocking)
    : pQueue(std::make_shared<LockingQueue<std::shared_ptr<ADatatype>>>(maxSize, blocking)),
      queue(*pQueue),
      pRunning(std::make_shared<std::atomic<bool>>(true)),
      running(*pRunning),
      pExceptionMessage(std::make_shared<std::string>("")),
      exceptionMessage(*pExceptionMessage),
      streamName(name) {
    // Copies of variables for the detached thread
    std::shared_ptr<std::atomic<bool>> pRunningCopy{pRunning};
    std::shared_ptr<std::string> pExceptionMessageCopy{pExceptionMessage};
    auto pQueueCopy = pQueue;

    // creates a thread which reads from connection into the queue
    readingThread = std::thread([name, pRunningCopy, conn, pExceptionMessageCopy, pQueueCopy]() {
        std::atomic<bool>& running = *pRunningCopy;
        LockingQueue<std::shared_ptr<ADatatype>>& queue = *pQueueCopy;

        std::uint64_t numPacketsRead = 0;
        try {
            // open stream with 1B write size (no writing will happen here)
            conn->openStream(name, 1);

            while(running) {
                // read packet
                auto* packet = conn->readFromStreamRaw(name);
                if(!running) break;

                // parse packet
                auto data = parsePacketToADatatype(packet);

                // Trace level debugging
                if(spdlog::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->getRaw()->serialize(metadata, type);
                    spdlog::trace("Received message from device - data size: {}, data type: {} data: {}", data->getRaw()->data.size(), type, nlohmann::json::from_msgpack(metadata).dump());
                }

                // release packet
                conn->readFromStreamRawRelease(name);

                // Add 'data' to queue
                queue.push(data);

                // Increment numPacketsRead
                numPacketsRead++;
            }

            conn->closeStream(name);

        } catch(const std::exception& ex) {
            *pExceptionMessageCopy = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        queue.destruct();
        running = false;
    });
}

DataOutputQueue::~DataOutputQueue() {
    // Set reading thread to stop
    running = false;

    // Destroy queue
    pQueue->destruct();
    pQueue = nullptr;

    // detach from thread, because currently no way to unblock underlying XLinkReadData
    // Make sure to not access DataOutputQueue fields from that thread anymore
    readingThread.detach();
}

std::string DataOutputQueue::getName() const {
    return streamName;
}

// DATA INPUT QUEUE
DataInputQueue::DataInputQueue(const std::shared_ptr<XLinkConnection>& conn, const std::string& name, unsigned int maxSize, bool blocking)
    : pQueue(std::make_shared<LockingQueue<std::shared_ptr<RawBuffer>>>(maxSize, blocking)),
      queue(*pQueue),
      pRunning(std::make_shared<std::atomic<bool>>(true)),
      running(*pRunning),
      pExceptionMessage(std::make_shared<std::string>("")),
      exceptionMessage(*pExceptionMessage),
      streamName(name) {
    // creates a thread which reads from connection into the queue
    std::shared_ptr<std::atomic<bool>> pRunningCopy{pRunning};
    std::shared_ptr<std::string> pExceptionMessageCopy{pExceptionMessage};
    auto pQueueCopy = pQueue;

    // do not capture 'this' as it this thread will outlive the parent object
    writingThread = std::thread([name, pRunningCopy, conn, pExceptionMessageCopy, pQueueCopy]() {
        std::atomic<bool>& running = *pRunningCopy;
        std::uint64_t numPacketsSent = 0;
        LockingQueue<std::shared_ptr<RawBuffer>>& queue = *pQueueCopy;

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
                    spdlog::trace("Sending message to device - data size: {}, data type: {} data: {}", data->data.size(), type, nlohmann::json::from_msgpack(metadata).dump());
                }

                // serialize
                auto serialized = serializeData(data);

                // Write packet to device
                conn->writeToStream(name, serialized);
                if(!running) return;

                // Increment num packets sent
                numPacketsSent++;
            }

            conn->closeStream(name);

        } catch(const std::exception& ex) {
            *pExceptionMessageCopy = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        queue.destruct();
        running = false;
    });
}

DataInputQueue::~DataInputQueue() {
    // Set writing thread to stop
    running = false;

    // Destroy queue
    pQueue->destruct();
    pQueue = nullptr;

    // detach thread to not block user space code
    // Make sure to not access DataInputQueue fields from that thread anymore
    writingThread.detach();
}

std::string DataInputQueue::getName() const {
    return streamName;
}

void DataInputQueue::send(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.push(val);
}
void DataInputQueue::send(const std::shared_ptr<ADatatype>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.push(val->serialize());
}
void DataInputQueue::send(const ADatatype& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.push(val.serialize());
}

void DataInputQueue::sendSync(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.waitEmpty();
    queue.push(val);
}
void DataInputQueue::sendSync(const std::shared_ptr<ADatatype>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.waitEmpty();
    queue.push(val->serialize());
}
void DataInputQueue::sendSync(const ADatatype& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.waitEmpty();
    queue.push(val.serialize());
}

}  // namespace dai

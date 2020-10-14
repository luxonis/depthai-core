#include "depthai/DataQueue.hpp"

#include <iostream>

#include "datatype/StreamPacketParser.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

namespace dai {

DataOutputQueue::DataOutputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool overwrite)
    : connection(std::move(conn)), pQueue(new LockingQueue<std::shared_ptr<RawBuffer>>(maxSize, overwrite)), queue(*pQueue),
    pRunning(std::make_shared<std::atomic<bool>>(true)), running(*pRunning) {
    // creates a thread which reads from connection into the queue
    readingThread = std::thread([this, streamName]() {
        std::shared_ptr<std::atomic<bool>> pRunningCopy{this->pRunning};
        std::atomic<bool>& running = *pRunningCopy;
        std::uint64_t numPacketsRead = 0;

        try {
            // open stream with 1B write size (no writing will happen here)
            connection->openStream(streamName, 1);

            while(running) {
                // read packet
                auto* packet = connection->readFromStreamRaw(streamName);
                if(!running) return;
                // parse packet
                auto data = parsePacket(packet);
                if(!running) return;
                // release packet
                connection->readFromStreamRawRelease(streamName);
                if(!running) return;
                // Add 'data' to queue
                queue.push(data);
                if(!running) return;
                // Increment numPacketsRead
                numPacketsRead++;
            }

            connection->closeStream(streamName);

        } catch(const std::exception& ex) {
            // TODO(themarpe)
            if(running) exceptionMessage = std::string(ex.what());
        }

        running = false;
    });
}

DataOutputQueue::~DataOutputQueue() {
    // Set reading thread to stop
    running = false;

    // Destroy queue
    pQueue = nullptr;

    // detach from thread, because currently no way to unblock underlying XLinkReadData
    // Make sure to not access DataOutputQueue fields from that thread anymore
    readingThread.detach();
}

DataInputQueue::DataInputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool overwrite)
    : connection(std::move(conn)), pQueue(new LockingQueue<std::shared_ptr<RawBuffer>>(maxSize, overwrite)), queue(*pQueue),
    pRunning(std::make_shared<std::atomic<bool>>(true)), running(*pRunning) {

    // creates a thread which reads from connection into the queue
    // Warning: if running is false, then thread must exit ASAP and can't access any this-> fields as parent object isn't available anymore
    writingThread = std::thread([this, streamName]() {
        std::shared_ptr<std::atomic<bool>> pRunningCopy{this->pRunning};
        std::atomic<bool>& running = *pRunningCopy;
        std::uint64_t numPacketsSent = 0;

        try {
            // open stream with 1B write size (no writing will happen here)
            connection->openStream(streamName, dai::XLINK_USB_BUFFER_MAX_SIZE);

            while(running) {
                // get data from queue
                std::shared_ptr<RawBuffer> data;
                if(!queue.waitAndPop(data)){
                    continue;
                }

                // serialize
                auto serialized = serializeData(data);

                // Write packet to device
                connection->writeToStream(streamName, serialized);
                if(!running) return;

                // Increment num packets sent
                numPacketsSent++;

            }

            if(running) connection->closeStream(streamName);

        } catch(const std::exception& ex) {
            // TODO(themarpe)
            if(running) exceptionMessage = std::string(ex.what());
        }

        running = false;
    });
}

DataInputQueue::~DataInputQueue() {
    // Set writing thread to stop
    running = false;

    // Destroy queue
    pQueue = nullptr;

    // detach from thread, because currently no way to unblock underlying XLinkWriteData
    // Make sure to not access DataInputQueue fields from that thread anymore
    writingThread.detach();

}

void DataInputQueue::send(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.push(val);
}

void DataInputQueue::sendSync(const std::shared_ptr<RawBuffer>& val) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.waitEmpty();
    queue.push(val);
}

}  // namespace dai

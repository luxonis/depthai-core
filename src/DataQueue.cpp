#include "depthai/DataQueue.hpp"

#include <iostream>

#include "depthai-shared/xlink/XLinkConstants.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/StreamPacketParser.hpp"

namespace dai {

DataOutputQueue::DataOutputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool overwrite)
    : pQueue(std::make_shared<LockingQueue<std::shared_ptr<ADatatype>>>(maxSize, overwrite)),
      queue(*pQueue),
      pRunning(std::make_shared<std::atomic<bool>>(true)),
      running(*pRunning),
      pExceptionMessage(std::make_shared<std::string>("")),
      exceptionMessage(*pExceptionMessage) {
    // Copies of variables for the detached thread
    std::shared_ptr<std::atomic<bool>> pRunningCopy{pRunning};
    std::shared_ptr<std::string> pExceptionMessageCopy{pExceptionMessage};
    auto pQueueCopy = pQueue;

    // creates a thread which reads from connection into the queue
    readingThread = std::thread([streamName, pRunningCopy, conn, pExceptionMessageCopy, pQueueCopy]() {
        std::atomic<bool>& running = *pRunningCopy;
        LockingQueue<std::shared_ptr<ADatatype>>& queue = *pQueueCopy;

        std::uint64_t numPacketsRead = 0;
        try {
            // open stream with 1B write size (no writing will happen here)
            conn->openStream(streamName, 1);

            while(running) {
                // read packet
                auto* packet = conn->readFromStreamRaw(streamName);
                if(!running) break;

                // parse packet
                auto data = parsePacketToADatatype(packet);

                // release packet
                conn->readFromStreamRawRelease(streamName);

                // Add 'data' to queue
                queue.push(data);

                // Increment numPacketsRead
                numPacketsRead++;
            }

            conn->closeStream(streamName);

        } catch(const std::exception& ex) {
            *pExceptionMessageCopy = std::string(ex.what());
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

DataInputQueue::DataInputQueue(std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool overwrite)
    : pQueue(std::make_shared<LockingQueue<std::shared_ptr<RawBuffer>>>(maxSize, overwrite)),
      queue(*pQueue),
      pRunning(std::make_shared<std::atomic<bool>>(true)),
      running(*pRunning),
      pExceptionMessage(std::make_shared<std::string>("")),
      exceptionMessage(*pExceptionMessage) {
    // creates a thread which reads from connection into the queue
    std::shared_ptr<std::atomic<bool>> pRunningCopy{pRunning};
    std::shared_ptr<std::string> pExceptionMessageCopy{pExceptionMessage};
    auto pQueueCopy = pQueue;

    // do not capture 'this' as it this thread will outlive the parent object
    writingThread = std::thread([streamName, pRunningCopy, conn, pExceptionMessageCopy, pQueueCopy]() {
        std::atomic<bool>& running = *pRunningCopy;
        std::uint64_t numPacketsSent = 0;
        LockingQueue<std::shared_ptr<RawBuffer>>& queue = *pQueueCopy;

        try {
            // open stream with 1B write size (no writing will happen here)
            conn->openStream(streamName, dai::XLINK_USB_BUFFER_MAX_SIZE);

            while(running) {
                // get data from queue
                std::shared_ptr<RawBuffer> data;
                if(!queue.waitAndPop(data)) {
                    continue;
                }

                // serialize
                auto serialized = serializeData(data);

                // Write packet to device
                conn->writeToStream(streamName, serialized);
                if(!running) return;

                // Increment num packets sent
                numPacketsSent++;
            }

            conn->closeStream(streamName);

        } catch(const std::exception& ex) {
            *pExceptionMessageCopy = std::string(ex.what());
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

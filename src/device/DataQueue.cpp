#include "depthai/device/DataQueue.hpp"

// std
#include <chrono>
#include <iostream>
#include <memory>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "pipeline/datatype/MessageGroup.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"

// shared
#include "depthai/xlink/XLinkConstants.hpp"

// libraries
#include "utility/Logging.hpp"

// Additions
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

namespace dai {

// DATA OUTPUT QUEUE
DataOutputQueue::DataOutputQueue(const std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool blocking)
    : queue(maxSize, blocking), name(streamName) {
    // Create stream first and then pass to thread
    // Open stream with 1B write size (no writing will happen here)
    XLinkStream stream(std::move(conn), name, 1);

    // Creates a thread which reads from connection into the queue
    readingThread = std::thread([this, stream = std::move(stream)]() mutable {
        std::uint64_t numPacketsRead = 0;
        try {
            while(running) {
                // Blocking -- parse packet and gather timing information
                auto packet = stream.readMove();
                const auto t1Parse = std::chrono::steady_clock::now();
                const auto msg = StreamMessageParser::parseMessage(std::move(packet));
                if(std::dynamic_pointer_cast<MessageGroup>(msg) != nullptr) {
                    auto msgGrp = std::static_pointer_cast<MessageGroup>(msg);
                    unsigned int size = msgGrp->getNumMessages();
                    std::vector<std::shared_ptr<ADatatype>> packets;
                    packets.reserve(size);
                    for(unsigned int i = 0; i < size; ++i) {
                        auto dpacket = stream.readMove();
                        packets.push_back(StreamMessageParser::parseMessage(&dpacket));
                    }
                    // TODO(Morato) bring back message groups
                    // for(auto& msg : msgGrp->group) {
                    //     msgGrp->add(msg.first, packets[msg.second.index]);
                    // }
                }
                const auto t2Parse = std::chrono::steady_clock::now();

                // Trace level debugging
                if(logger::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    msg->serialize(metadata, type);
                    logger::trace("Received message from device ({}) - parsing time: {}, data size: {}, object type: {} object data: {}",
                                  name,
                                  std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                                  msg->data->getSize(),
                                  static_cast<std::int32_t>(type),
                                  spdlog::to_hex(metadata));
                }

                // Add 'data' to queue
                if(!queue.push(msg)) {
                    throw std::runtime_error(fmt::format("Underlying queue destructed"));
                }

                // Increment numPacketsRead
                numPacketsRead++;

                // Call callbacks
                {
                    std::unique_lock<std::mutex> l(callbacksMtx);
                    for(const auto& kv : callbacks) {
                        const auto& callback = kv.second;
                        try {
                            callback(name, msg);
                        } catch(const std::exception& ex) {
                            logger::error("Callback with id: {} throwed an exception: {}", kv.first, ex.what());
                        }
                    }
                }
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        close();
    });
}

// This function is thread-unsafe. The idea of "isClosed" is ephemerial and
// since there is no mutex lock, its state is outdated and invalid even before
// the logical NOT in this function. This calculated boolean then continues to degrade
// in validity as it is returned by value to the caller
bool DataOutputQueue::isClosed() const {
    return !running;
}

void DataOutputQueue::close() {
    // Set reading thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if((readingThread.get_id() != std::this_thread::get_id()) && readingThread.joinable()) readingThread.join();

    // Log
    logger::debug("DataOutputQueue ({}) closed", name);
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

    // move assign callback
    callbacks[id] = std::move(callback);

    // return id assigned to the callback
    return id;
}

int DataOutputQueue::addCallback(std::function<void(std::shared_ptr<ADatatype>)> callback) {
    // Create a wrapper
    return addCallback([callback = std::move(callback)](std::string, std::shared_ptr<ADatatype> message) { callback(std::move(message)); });
}

int DataOutputQueue::addCallback(std::function<void()> callback) {
    // Create a wrapper
    return addCallback([callback = std::move(callback)](std::string, std::shared_ptr<ADatatype>) { callback(); });
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
DataInputQueue::DataInputQueue(
    const std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool blocking, std::size_t maxDataSize)
    : queue(maxSize, blocking), name(streamName), maxDataSize(maxDataSize) {
    // open stream with maxDataSize write size
    XLinkStream stream(std::move(conn), name, maxDataSize + device::XLINK_MESSAGE_METADATA_MAX_SIZE);

    writingThread = std::thread([this, stream = std::move(stream)]() mutable {
        using namespace std::chrono;
        std::uint64_t numPacketsSent = 0;
        try {
            while(running) {
                // get data from queue
                OutgoingMessage outgoing;
                if(!queue.waitAndPop(outgoing)) {
                    continue;
                }

                // Blocking
                auto t1 = steady_clock::now();
                if(outgoing.data->getSize() > 0) {
                    stream.write(outgoing.data->getData(), outgoing.metadata);
                } else {
                    stream.write(outgoing.metadata);
                }
                auto t2 = steady_clock::now();
                // Log
                if(spdlog::get_level() == spdlog::level::trace) {
                    logger::trace("Sent message to device ({}) - data size: {}, metadata: {}, sending time: {}",
                                  stream.getStreamName(),
                                  outgoing.data->getSize(),
                                  spdlog::to_hex(outgoing.metadata),
                                  duration_cast<microseconds>(t2 - t1));
                }
                // TODO Bring back support for MessageGroup
                //                 if(std::dynamic_pointer_cast<MessageGroup>())
                //                 // serialize
                //                 auto t1Parse = std::chrono::steady_clock::now();
                //                 std::vector<std::vector<uint8_t>> serializedAux;
                //                 if(data->getType() == DatatypeEnum::MessageGroup) {
                //                     auto rawMsgGrp = std::dynamic_pointer_cast<RawMessageGroup>(data);
                //                     serializedAux.reserve(rawMsgGrp->group.size());
                //                     unsigned int index = 0;
                //                     for(auto& msg : rawMsgGrp->group) {
                //                         msg.second.index = index++;
                //                         serializedAux.push_back(StreamMessageParser::serializeMessage(msg.second.buffer));
                //                     }
                //                 }
                //                 auto serialized = StreamMessageParser::serializeMessage(data);
                //                 auto t2Parse = std::chrono::steady_clock::now();

                //                 // Trace level debugging
                //                 if(logger::get_level() == spdlog::level::trace) {
                //                     std::vector<std::uint8_t> metadata;
                //                     DatatypeEnum type;
                //                     data->serialize(metadata, type);
                //                     logger::trace("Sending message to device ({}) - serialize time: {}, data size: {}, object type: {} object data: {}",
                //                                   name,
                //                                   std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                //                                   data->data.size(),
                //                                   type,
                //                                   spdlog::to_hex(metadata));
                //                 }

                //                 // Blocking
                //                 stream.write(serialized);
                //                 for(auto& msg : serializedAux) {
                //                     stream.write(msg);
                // }

                // Increment num packets sent
                numPacketsSent++;
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        close();
    });
}

// This function is thread-unsafe. The idea of "isClosed" is ephemerial and
// since there is no mutex lock, its state is outdated and invalid even before
// the logical NOT in this function. This calculated boolean then continues to degrade
// in validity as it is returned by value to the caller
bool DataInputQueue::isClosed() const {
    return !running;
}

void DataInputQueue::close() {
    // Set writing thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if((writingThread.get_id() != std::this_thread::get_id()) && writingThread.joinable()) writingThread.join();

    // Log
    logger::debug("DataInputQueue ({}) closed", name);
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

// BUGBUG https://github.com/luxonis/depthai-core/issues/762
void DataInputQueue::setMaxDataSize(std::size_t maxSize) {
    maxDataSize = maxSize;
}

std::size_t DataInputQueue::getMaxDataSize() {
    return maxDataSize;
}

std::string DataInputQueue::getName() const {
    return name;
}

DataInputQueue::OutgoingMessage DataInputQueue::getOutgoingMessage(const ADatatype& message) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    // Check if stream receiver has enough space for this message
    if(message.data->getSize() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", message.data->getSize(), maxDataSize));
    }

    // TODO(themarpe) - move serialization to be the last step
    // Create outgoing message and serialize
    OutgoingMessage outgoing;
    // serialize
    outgoing.data = message.data;
    outgoing.metadata = StreamMessageParser::serializeMetadata(message);
    return outgoing;
}

void DataInputQueue::send(const std::shared_ptr<ADatatype>& message) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!message) throw std::invalid_argument("Message passed is not valid (nullptr)");
    DataInputQueue::send(*message);
}

void DataInputQueue::send(const ADatatype& message) {
    OutgoingMessage outgoing = getOutgoingMessage(message);

    if(!queue.push(std::move(outgoing))) {
        throw std::runtime_error(fmt::format("Underlying queue destructed"));
    }
}

bool DataInputQueue::send(const ADatatype& message, std::chrono::milliseconds timeout) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    OutgoingMessage outgoing = getOutgoingMessage(message);

    return queue.tryWaitAndPush(std::move(outgoing), timeout);
}

bool DataInputQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    return send(*msg, timeout);
}
}  // namespace dai

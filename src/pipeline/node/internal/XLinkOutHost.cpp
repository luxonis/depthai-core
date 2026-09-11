#include "depthai/pipeline/node/internal/XLinkOutHost.hpp"

#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

// libraries
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "utility/Logging.hpp"
#include "utility/SharedMemory.hpp"

namespace dai {
namespace node {
namespace internal {
// XLinkInHost::XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName) : conn(std::move(conn)), streamName(std::move(streamName)){};

void XLinkOutHost::setStreamName(const std::string& name) {
    streamName = name;
}

void XLinkOutHost::setConnection(std::shared_ptr<XLinkConnection> conn) {
    std::lock_guard<std::mutex> lock(mtx);
    this->conn = std::move(conn);
    connectionRefreshed = true;
    isWaitingForReconnect.notify_all();
}

void XLinkOutHost::disconnect() {
    std::lock_guard<std::mutex> lock(mtx);
    isDisconnected = true;
    isWaitingForReconnect.notify_all();
}

void XLinkOutHost::allowStreamResize(bool allow) {
    allowResize = allow;
}

void XLinkOutHost::run() {
    {
        // Consume a connection refresh recorded before the node started (build time)
        std::lock_guard<std::mutex> lock(mtx);
        connectionRefreshed = false;
    }
    // // Create a stream for the connection
    // TODO(Morato) - automatically increase the buffer size lazily
    bool reconnect = true;
    while(reconnect) {
        reconnect = false;
        auto currentMaxSize = device::XLINK_USB_BUFFER_MAX_SIZE + device::XLINK_MESSAGE_METADATA_MAX_SIZE;
        // Copy under the lock - setConnection can rebind concurrently
        std::shared_ptr<XLinkConnection> currentConn;
        {
            std::lock_guard<std::mutex> lock(mtx);
            currentConn = conn;
        }
        std::unique_ptr<XLinkStream> streamPtr;
        try {
            streamPtr = std::make_unique<XLinkStream>(currentConn, streamName, currentMaxSize);
        } catch(const std::exception& ex) {
            // Connection unusable (e.g. closed while waking up) - park until it is
            // refreshed or the device is declared gone
            logger::error("Cannot open stream '{}': {}", streamName, ex.what());
            std::unique_lock<std::mutex> lck(mtx);
            isWaitingForReconnect.wait(lck, [this]() { return isDisconnected || connectionRefreshed; });
            if(isDisconnected) {
                logger::warn("XLinkOutHost '{}' stopping - device connection was lost", streamName);
                return;
            }
            connectionRefreshed = false;
            reconnect = true;
            continue;
        }
        XLinkStream& stream = *streamPtr;
        // File descriptors are only valid across a local shared-memory transport;
        // any other destination gets the mapped bytes instead (one copy)
        const bool destinationIsLocalShdmem = currentConn != nullptr && currentConn->getDeviceInfo().protocol == X_LINK_LOCAL_SHDMEM;
        auto increaseBufferSize = [&stream, &currentMaxSize, &currentConn, this](const std::size_t& maxSize) {
            if(!this->allowResize) {
                logger::error("Data size exceeds the maximum buffer size - please increase the buffer size");
                throw std::runtime_error("Data size exceeds the maximum buffer size");
            }
            stream = XLinkStream(currentConn, this->streamName, maxSize);
            currentMaxSize = maxSize;
        };
        while(mainLoop()) {
            try {
                std::shared_ptr<ADatatype> outgoing;
                {
                    auto blockEvent = this->inputBlockEvent();
                    outgoing = in.get();
                }
                auto metadata = StreamMessageParser::serializeMetadata(outgoing);

                using namespace std::chrono;
                // Blocking
                auto t1 = steady_clock::now();
                auto outgoingDataSize = outgoing->data->getSize();
                if(outgoingDataSize > currentMaxSize - metadata.size()) {
                    increaseBufferSize(outgoingDataSize + metadata.size());
                }
                if(outgoing->data->getSize() > 0) {
                    auto sharedMemory = std::dynamic_pointer_cast<SharedMemory>(outgoing->data);
                    if(sharedMemory && sharedMemory->getFd() > 0 && destinationIsLocalShdmem) {
                        stream.write(sharedMemory->getFd(), metadata);
                    } else {
                        stream.write(outgoing->data->getData(), metadata);
                    }
                } else {
                    stream.write(metadata);
                }
                auto t2 = steady_clock::now();
                // Log
                if(spdlog::get_level() == spdlog::level::trace) {
                    logger::trace("Sent message to device ({}) - data size: {}, metadata: {}, sending time: {}",
                                  stream.getStreamName(),
                                  outgoing->data->getSize(),
                                  spdlog::to_hex(metadata),
                                  duration_cast<microseconds>(t2 - t1));
                }

                // Attempt dynamic cast to MessageGroup
                if(auto msgGroupPtr = std::dynamic_pointer_cast<MessageGroup>(outgoing)) {
                    logger::trace("Sending group message to device with {} messages", msgGroupPtr->group.size());
                    for(auto& msg : msgGroupPtr->group) {
                        logger::trace("Sending part of a group message: {}", msg.first);
                        auto metadata = StreamMessageParser::serializeMetadata(msg.second);
                        outgoingDataSize = msg.second->data->getSize();
                        if(outgoingDataSize > currentMaxSize - metadata.size()) {
                            increaseBufferSize(outgoingDataSize + metadata.size());
                        }
                        if(msg.second->data->getSize() > 0) {
                            stream.write(msg.second->data->getData(), metadata);
                        } else {
                            stream.write(metadata);
                        }
                    }
                }
            } catch(const std::exception& ex) {
                if(isRunning()) {
                    logger::error("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
                    std::unique_lock<std::mutex> lck(mtx);
                    logger::info("Waiting for reconnect (XLINKOUTHOST)\n");
                    isWaitingForReconnect.wait(lck, [this]() { return isDisconnected || connectionRefreshed; });
                    if(isDisconnected) {
                        // Device is gone for good - exit quietly so this stream stops
                        // instead of tearing down the whole pipeline
                        logger::warn("XLinkOutHost '{}' stopping - device connection was lost", streamName);
                        return;
                    }
                    connectionRefreshed = false;
                    logger::info("Reconnected (XLINKOUTHOST)\n");
                    reconnect = true;
                    break;
                } else {
                    // If the node is not running, we can safely ignore the exception, since it's expected
                    logger::info("XLinkOutHost node stopped - exception: {}", ex.what());
                    break;
                }
            }
        }
    }
}

}  // namespace internal
}  // namespace node
}  // namespace dai

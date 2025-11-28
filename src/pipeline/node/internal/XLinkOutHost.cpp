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
    this->conn = std::move(conn);
    std::lock_guard<std::mutex> lock(mtx);
    isWaitingForReconnect.notify_all();
}

void XLinkOutHost::disconnect() {
    isDisconnected = true;
    std::lock_guard<std::mutex> lock(mtx);
    isWaitingForReconnect.notify_all();
}

void XLinkOutHost::allowStreamResize(bool allow) {
    allowResize = allow;
}

void XLinkOutHost::run() {
    // // Create a stream for the connection
    // TODO(Morato) - automatically increase the buffer size lazily
    bool reconnect = true;
    while(reconnect) {
        reconnect = false;
        auto currentMaxSize = device::XLINK_USB_BUFFER_MAX_SIZE + device::XLINK_MESSAGE_METADATA_MAX_SIZE;
        XLinkStream stream(conn, streamName, currentMaxSize);
        auto increaseBufferSize = [&stream, &currentMaxSize, this](const std::size_t& maxSize) {
            if(!this->allowResize) {
                logger::error("Data size exceeds the maximum buffer size - please increase the buffer size");
                throw std::runtime_error("Data size exceeds the maximum buffer size");
            }
            stream = XLinkStream(this->conn, this->streamName, maxSize);
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
                {
                    auto blockEvent = this->outputBlockEvent();
                    if(outgoing->data->getSize() > 0) {
                        auto sharedMemory = std::dynamic_pointer_cast<SharedMemory>(outgoing->data);
                        if(sharedMemory && sharedMemory->getFd() > 0) {
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
                }
            } catch(const std::exception& ex) {
                if(isRunning()) {
                    auto exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
                    std::unique_lock<std::mutex> lck(mtx);
                    logger::info("Waiting for reconnect (XLINKOUTHOST)\n");
                    isWaitingForReconnect.wait(lck);
                    if(isDisconnected) throw std::runtime_error(exceptionMessage);
                    logger::info("Reconnected (XLINKOUTHOST)\n");
                    reconnect = true;
                    break;
                } else {
                    // If the node is not running, we can safely ignore the exception, since it's expected
                    logger::info("XLinkInHost node stopped - exception: {}", ex.what());
                    break;
                }
            }
        }
    }
}

}  // namespace internal
}  // namespace node
}  // namespace dai

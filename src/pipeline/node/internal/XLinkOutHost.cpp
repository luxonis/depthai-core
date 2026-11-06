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

namespace {
// A stream failure on a connection that nobody closes within this period is not a device
// loss (the device monitor closes the connection a couple of watchdog periods after one)
constexpr auto CONNECTION_LOSS_GRACE = std::chrono::seconds(10);
constexpr auto PARK_POLL_INTERVAL = std::chrono::milliseconds(100);
}  // namespace

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

bool XLinkOutHost::parkUntilReconnect(const std::shared_ptr<XLinkConnection>& lostConn, bool streamOpenFailed) {
    using namespace std::chrono;
    const auto parkStart = steady_clock::now();
    std::unique_lock<std::mutex> lck(mtx);
    while(isRunning() && !isDisconnected && !connectionRefreshed) {
        const bool connectionAlive = lostConn != nullptr && !lostConn->isClosed();
        if(connectionAlive && steady_clock::now() - parkStart > CONNECTION_LOSS_GRACE) {
            // Not a device loss: the connection is healthy but the stream failed (e.g. the
            // device refused to open it). Surface the error instead of idling forever.
            throw std::runtime_error(fmt::format(
                "XLinkOutHost '{}': stream {} although the device connection is healthy", streamName, streamOpenFailed ? "could not be opened" : "failed"));
        }
        isWaitingForReconnect.wait_for(lck, PARK_POLL_INTERVAL);
        // Discard whatever the producers sent meanwhile so they never block on this dead stream
        lck.unlock();
        in.tryGetAll();
        lck.lock();
    }
    if(!isRunning()) {
        return false;
    }
    if(isDisconnected) {
        lck.unlock();
        // Device is gone for good - keep the input flowing (and discarded) so the producers
        // linked to it keep serving their other consumers, until the node is stopped
        logger::warn("XLinkOutHost '{}' idling - device connection was lost", streamName);
        drainUntilStopped();
        return false;
    }
    connectionRefreshed = false;
    return true;
}

void XLinkOutHost::drainUntilStopped() {
    while(mainLoop()) {
        try {
            in.get();
        } catch(const MessageQueue::QueueException&) {
            // stop() closed the input
            break;
        }
    }
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
            if(!parkUntilReconnect(currentConn, true)) {
                return;
            }
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
                    logger::info("Waiting for reconnect (XLINKOUTHOST)\n");
                    // Device gone for good: the stream stays idle instead of tearing down the pipeline
                    if(!parkUntilReconnect(currentConn, false)) {
                        return;
                    }
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

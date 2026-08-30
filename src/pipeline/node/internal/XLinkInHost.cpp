#include "depthai/pipeline/node/internal/XLinkInHost.hpp"

#include "../../datatype/PacketizedData.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

// libraries
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {
namespace internal {
// XLinkInHost::XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName) : conn(std::move(conn)), streamName(std::move(streamName)){};

void XLinkInHost::setStreamName(const std::string& name) {
    streamName = name;
}

void XLinkInHost::setConnection(std::shared_ptr<XLinkConnection> conn) {
    std::lock_guard<std::mutex> lock(mtx);
    this->conn = std::move(conn);
    connectionRefreshed = true;
    isWaitingForReconnect.notify_all();
}

void XLinkInHost::disconnect() {
    std::lock_guard<std::mutex> lock(mtx);
    isDisconnected = true;
    isWaitingForReconnect.notify_all();
}

StreamPacketDesc XLinkInHost::readStreamMessage() const {
    return stream->readMove();
}

std::shared_ptr<ADatatype> XLinkInHost::readData() const {
    auto packet = readStreamMessage();
    const auto msg = StreamMessageParser::parseMessage(std::move(packet));
    if(auto messageGroup = std::dynamic_pointer_cast<MessageGroup>(msg)) {
        parseMessageGroup(messageGroup);
    }
    if(auto packetizedData = std::dynamic_pointer_cast<PacketizedData>(msg)) {
        return parsePacketizedData(packetizedData);
    }
    return msg;
}

std::shared_ptr<ADatatype> XLinkInHost::parsePacketizedData(const std::shared_ptr<PacketizedData>& packetizedData) const {
    std::vector<std::uint8_t> payload;
    payload.reserve(packetizedData->totalSize);
    std::uint32_t currentSize = 0;

    for(std::uint32_t i = 0; i < packetizedData->numPackets; ++i) {
        auto packet = readStreamMessage();
        if(packet.length == 0) {
            continue;
        }
        currentSize += packet.length;
        payload.insert(payload.end(), packet.data, packet.data + packet.length);
    }

    if(currentSize != packetizedData->totalSize) {
        throw std::runtime_error(
            "XLinkInHost: Data size mismatch. "
            "Expected size: "
            + std::to_string(packetizedData->totalSize) + ", but received size: " + std::to_string(currentSize) + ".");
    }

    streamPacketDesc_t packet{};
    packet.data = payload.data();
    packet.length = static_cast<uint32_t>(payload.size());
    packet.fd = -1;

    return StreamMessageParser::parseMessage(&packet);
}

void XLinkInHost::parseMessageGroup(const std::shared_ptr<MessageGroup>& messageGroup) const {
    for(auto& msg : messageGroup->group) {
        msg.second = readData();
    }
}

void XLinkInHost::run() {
    {
        // Consume a connection refresh recorded before the node started (build time)
        std::lock_guard<std::mutex> lock(mtx);
        connectionRefreshed = false;
    }
    // Create a stream for the connection
    bool reconnect = true;
    while(reconnect) {
        reconnect = false;
        try {
            // Copy under the lock - setConnection can rebind concurrently
            std::shared_ptr<XLinkConnection> currentConn;
            {
                std::lock_guard<std::mutex> lock(mtx);
                currentConn = conn;
            }
            stream = std::make_unique<XLinkStream>(std::move(currentConn), streamName, 1);
        } catch(const std::exception& ex) {
            // Connection unusable (e.g. closed while waking up) - park until it is
            // refreshed or the device is declared gone
            logger::error("Cannot open stream '{}': {}", streamName, ex.what());
            std::unique_lock<std::mutex> lck(mtx);
            isWaitingForReconnect.wait(lck, [this]() { return isDisconnected || connectionRefreshed; });
            if(isDisconnected) {
                logger::warn("XLinkInHost '{}' stopping - device connection was lost", streamName);
                return;
            }
            connectionRefreshed = false;
            reconnect = true;
            continue;
        }
        while(mainLoop()) {
            try {
                // Blocking -- parse packet and gather timing information
                const auto t1Parse = std::chrono::steady_clock::now();
                auto msg = readData();
                const auto t2Parse = std::chrono::steady_clock::now();

                // Trace level debugging
                if(logger::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    msg->serialize(metadata, type);
                    logger::trace("Received message from device ({}) - parsing time: {}, data size: {}, object type: {} object data: {}",
                                  streamName,
                                  std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                                  msg->data->getSize(),
                                  static_cast<std::int32_t>(type),
                                  spdlog::to_hex(metadata));
                }

                {
                    auto blockEvent = this->outputBlockEvent();
                    out.send(msg);
                }
                // // Add 'data' to queue
                // if(!queue.push(msg)) {
                //     throw std::runtime_error(fmt::format("Underlying queue destructed"));
                // }

                // Call callbacks
                // {
                //     std::unique_lock<std::mutex> l(callbacksMtx);
                //     for(const auto& kv : callbacks) {
                //         const auto& callback = kv.second;
                //         try {
                //             callback(name, msg);
                //         } catch(const std::exception& ex) {
                //             logger::error("Callback with id: {} throwed an exception: {}", kv.first, ex.what());
                //         }
                //     }
                // }
            } catch(const dai::MessageQueue::QueueException& ex) {
                logger::info("XLinkInHost node stopped - exception: {}", ex.what());
                break;
            } catch(const std::exception& ex) {
                if(isRunning()) {
                    auto exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
                    logger::error(exceptionMessage);
                    std::unique_lock<std::mutex> lck(mtx);
                    logger::info("Waiting for reconnect (XLINKINHOST)\n");
                    isWaitingForReconnect.wait(lck, [this]() { return isDisconnected || connectionRefreshed; });
                    if(isDisconnected) {
                        // Device is gone for good - exit quietly so downstream inputs
                        // go idle instead of tearing down the whole pipeline
                        logger::warn("XLinkInHost '{}' stopping - device connection was lost", streamName);
                        return;
                    }
                    connectionRefreshed = false;
                    logger::info("Reconnected (XLINKINHOST)\n");
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

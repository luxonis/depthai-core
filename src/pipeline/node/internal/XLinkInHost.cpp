#include "depthai/pipeline/node/internal/XLinkInHost.hpp"

#include "../../datatype/PacketizedData.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {
namespace internal {

namespace {
constexpr auto CONNECTION_LOSS_GRACE = std::chrono::seconds(10);  // Open after this = not a device loss: the monitor closes lost ones in ~2 watchdog periods
constexpr auto PARK_POLL_INTERVAL = std::chrono::milliseconds(100);
}  // namespace

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

bool XLinkInHost::parkUntilReconnect(const std::shared_ptr<XLinkConnection>& lostConn, bool streamOpenFailed) {
    using namespace std::chrono;
    const auto parkStart = steady_clock::now();
    std::unique_lock<std::mutex> lck(mtx);
    while(isRunning() && !isDisconnected && !connectionRefreshed) {
        const bool connectionAlive = lostConn != nullptr && !lostConn->isClosed();
        if(connectionAlive && steady_clock::now() - parkStart > CONNECTION_LOSS_GRACE) {
            throw std::runtime_error(fmt::format(
                "XLinkInHost '{}': stream {} although the device connection is healthy", streamName, streamOpenFailed ? "could not be opened" : "failed"));
        }
        isWaitingForReconnect.wait_for(lck, PARK_POLL_INTERVAL);
    }
    if(!isRunning()) {
        return false;
    }
    if(isDisconnected) {
        logger::warn("XLinkInHost '{}' stopping - device connection was lost", streamName);  // Return quietly: downstream goes idle, pipeline stays up
        return false;
    }
    connectionRefreshed = false;
    return true;
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

    return StreamMessageParser::parseMessage(std::move(payload));
}

void XLinkInHost::parseMessageGroup(const std::shared_ptr<MessageGroup>& messageGroup) const {
    for(auto& msg : messageGroup->group) {
        msg.second = readData();
    }
}

void XLinkInHost::run() {
    {
        std::lock_guard<std::mutex> lock(mtx);  // Drop a refresh recorded at build time, before the node started
        connectionRefreshed = false;
    }
    bool reconnect = true;
    while(reconnect) {
        reconnect = false;
        std::shared_ptr<XLinkConnection> currentConn;  // Copy under the lock: setConnection can rebind concurrently
        {
            std::lock_guard<std::mutex> lock(mtx);
            currentConn = conn;
        }
        try {
            stream = std::make_unique<XLinkStream>(currentConn, streamName, 1);
        } catch(const std::exception& ex) {
            logger::error("Cannot open stream '{}': {}", streamName, ex.what());
            if(!parkUntilReconnect(currentConn, true)) {
                return;
            }
            reconnect = true;
            continue;
        }
        while(mainLoop()) {
            try {
                const auto t1Parse = std::chrono::steady_clock::now();
                auto msg = readData();
                const auto t2Parse = std::chrono::steady_clock::now();

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
            } catch(const dai::MessageQueue::QueueException& ex) {
                logger::info("XLinkInHost node stopped - exception: {}", ex.what());
                break;
            } catch(const std::exception& ex) {
                if(isRunning()) {
                    auto exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
                    logger::error(exceptionMessage);
                    logger::info("Waiting for reconnect (XLINKINHOST)\n");
                    if(!parkUntilReconnect(currentConn, false)) {
                        return;
                    }
                    logger::info("Reconnected (XLINKINHOST)\n");
                    reconnect = true;
                    break;
                } else {
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

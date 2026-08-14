#include "depthai/pipeline/node/internal/XLinkInHost.hpp"

#include "../../datatype/PacketizedData.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

// libraries
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {
namespace internal {

namespace {

/// Name the device a transformation is expressed in, unless the message already knows it.
void qualifyReferenceFrame(ImgTransformation& transformation, const std::string& deviceId) {
    if(!transformation.isValid()) return;
    auto extrinsics = transformation.getExtrinsics();
    if(extrinsics.toDeviceId.has_value()) return;
    extrinsics.toDeviceId = deviceId;
    transformation.setExtrinsics(extrinsics);
}

void qualifyReferenceFrame(std::optional<ImgTransformation>& transformation, const std::string& deviceId) {
    if(transformation.has_value()) qualifyReferenceFrame(*transformation, deviceId);
}

/**
 * Devices do not know their own MXID in the messages they produce - the field is not even part of the protocol - so
 * the host names the device every message came from. That is what makes the frames of several devices tellable apart
 * downstream, e.g. in CoordinateFrameTransform.
 */
void qualifyReferenceFrame(const std::shared_ptr<ADatatype>& message, const std::string& deviceId) {
    if(message == nullptr || deviceId.empty()) return;
    if(auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(message)) {
        qualifyReferenceFrame(imgFrame->getTransformation(), deviceId);
    } else if(auto encodedFrame = std::dynamic_pointer_cast<EncodedFrame>(message)) {
        qualifyReferenceFrame(encodedFrame->transformation, deviceId);
    } else if(auto transformable = std::dynamic_pointer_cast<Transformable>(message)) {
        qualifyReferenceFrame(transformable->transformation, deviceId);
    } else if(auto nnData = std::dynamic_pointer_cast<NNData>(message)) {
        qualifyReferenceFrame(nnData->transformation, deviceId);
    } else if(auto annotations = std::dynamic_pointer_cast<ImgAnnotations>(message)) {
        qualifyReferenceFrame(annotations->transformation, deviceId);
    }
}

}  // namespace
// XLinkInHost::XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName) : conn(std::move(conn)), streamName(std::move(streamName)){};

void XLinkInHost::setStreamName(const std::string& name) {
    streamName = name;
}

void XLinkInHost::setDeviceId(const std::string& id) {
    deviceId = id;
}

void XLinkInHost::setConnection(std::shared_ptr<XLinkConnection> conn) {
    this->conn = std::move(conn);
    std::lock_guard<std::mutex> lock(mtx);
    isWaitingForReconnect.notify_all();
}

void XLinkInHost::disconnect() {
    isDisconnected = true;
    std::lock_guard<std::mutex> lock(mtx);
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
        auto data = parsePacketizedData(packetizedData);
        qualifyReferenceFrame(data, deviceId);
        return data;
    }
    qualifyReferenceFrame(msg, deviceId);
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
    // Create a stream for the connection
    bool reconnect = true;
    while(reconnect) {
        reconnect = false;
        stream = std::make_unique<XLinkStream>(std::move(conn), streamName, 1);
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
                    isWaitingForReconnect.wait(lck);
                    if(isDisconnected) throw std::runtime_error(exceptionMessage);
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

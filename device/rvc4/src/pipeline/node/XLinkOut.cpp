#include "XLinkOut.hpp"

#include "depthai/xlink/XLinkConstants.hpp"

#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>
#include <depthai/utility/Memory.hpp>
#include <depthai/utility/SharedMemory.hpp>
#include <depthai/utility/VectorMemory.hpp>
#include <memory>

#include "pipeline/datatype/PacketizedData.hpp"
#include <XLink/XLink.h>

#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "pipeline/NodeTiming.hpp"

namespace dai {
namespace gate {

const bool PRINT_DEBUG = false;
const std::chrono::milliseconds DELAY_AFTER_XLINK_CONNECTION{50};

XLinkError_t XLinkOut::writeData(streamId_t streamId, const uint8_t* data, size_t size) const {
    return XLinkWriteData(streamId, data, size);
}

XLinkError_t XLinkOut::writeData(streamId_t streamId, dai::span<const uint8_t> data, dai::span<const uint8_t> metadata) const {
    return XLinkWriteData2(streamId, data.data(), data.size(), metadata.data(), metadata.size());
}

XLinkError_t XLinkOut::writeData(streamId_t streamId, dai::span<const uint8_t> metadata) const {
    return XLinkWriteData(streamId, metadata.data(), metadata.size());
}

XLinkError_t XLinkOut::sendPacketizedFrame(streamId_t streamId, dai::span<const uint8_t> data, dai::span<const uint8_t> metadata) const {
    const size_t dataSize = data.size();
    const size_t metadataSize = metadata.size();
    const size_t combinedSize = dataSize + metadataSize;

    size_t packetSize = properties.packetSize > 0 ? static_cast<size_t>(properties.packetSize) : combinedSize;

    if(properties.packetSize <= 0) {
        return writeData(streamId, data, metadata);
    }

    float secondsPerPacket = properties.bytesPerSecondLimit > 0 ? static_cast<float>(packetSize) / properties.bytesPerSecondLimit : 0.0f;
    size_t numPackets = (combinedSize + packetSize - 1) / packetSize;

    dai::PacketizedData pcktData(numPackets, combinedSize);
    std::vector<uint8_t> xLinkMetaBuffer = StreamMessageParser::serializeMetadata(pcktData);
    XLinkError_t status = writeData(streamId, xLinkMetaBuffer);
    if(status != X_LINK_SUCCESS) return status;

    size_t sentBytes = 0;
    for(size_t i = 0; i < numPackets; ++i) {
        auto loopStart = std::chrono::steady_clock::now();
        size_t thisPacketSize = std::min<size_t>(packetSize, combinedSize - sentBytes);

        if(sentBytes + thisPacketSize <= dataSize) {
            status = writeData(streamId, data.data() + sentBytes, thisPacketSize);
        } else if(sentBytes >= dataSize) {
            status = writeData(streamId, metadata.data() + (sentBytes - dataSize), thisPacketSize);
        } else {
            size_t fromData = dataSize - sentBytes;
            size_t fromMeta = thisPacketSize - fromData;
            std::vector<uint8_t> splitBuffer;
            splitBuffer.reserve(thisPacketSize);
            splitBuffer.insert(splitBuffer.end(), data.data() + sentBytes, data.data() + dataSize);
            splitBuffer.insert(splitBuffer.end(), metadata.data(), metadata.data() + fromMeta);
            status = writeData(streamId, splitBuffer.data(), splitBuffer.size());
        }

        if(status != X_LINK_SUCCESS) return status;
        sentBytes += thisPacketSize;

        if(secondsPerPacket > 0) {
            auto timeSpent = std::chrono::steady_clock::now() - loopStart;
            auto willSleepFor = std::chrono::duration<float>(secondsPerPacket) - timeSpent;
            if(willSleepFor > std::chrono::duration<float>::zero()) {
                std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(willSleepFor));
            }
        }
    }

    return X_LINK_SUCCESS;
}

bool XLinkOut::shouldSkipFrame(double maxFps,
                               std::chrono::steady_clock::time_point& tPrevPacket,
                               std::chrono::steady_clock::time_point& tPrevSecond,
                               int& packetCounter) const {
    if(maxFps <= 0.0) return false;

    using namespace std::chrono;
    auto currentTime = steady_clock::now();

    if(currentTime - tPrevSecond > 1s) {
        packetCounter = 0;
        tPrevSecond = currentTime;
    }

    auto minPeriod = duration<double>(1.0 / (maxFps * 1.05));
    if((currentTime - tPrevPacket) < minPeriod || packetCounter >= std::ceil(maxFps)) {
        return true;
    }
    return false;
}

XLinkError_t XLinkOut::transmitMessage(streamId_t& streamId,
                                       std::shared_ptr<dai::ADatatype> message,
                                       size_t& currentStreamSize,
                                       const std::string& streamName) const {
    auto metadata = StreamMessageParser::serializeMetadata(message);
    auto dataSize = message->data->getSize();

    ensureStreamSize(streamId, currentStreamSize, message->data->getSize(), streamName);

    if(auto msgGroupPtr = std::dynamic_pointer_cast<dai::MessageGroup>(message)) {
        XLinkError_t status = writeData(streamId, metadata);
        for(auto& pair : msgGroupPtr->group) {
            if(status != X_LINK_SUCCESS) break;
            status = transmitMessage(streamId, pair.second, currentStreamSize, streamName);  // Recursive for group parts
        }
        return status;
    }

    if(dataSize == 0 || properties.metadataOnly) {
        return writeData(streamId, metadata);
    }
    return sendPacketizedFrame(streamId, message->data->getData(), metadata);
}

void XLinkOut::ensureStreamSize(streamId_t& streamId, size_t& currentStreamSize, size_t requiredSize, const std::string& name) const {
    constexpr int metadataMaxSize = dai::device::XLINK_MESSAGE_METADATA_MAX_SIZE;
    size_t totalNeeded = requiredSize + metadataMaxSize;

    if(totalNeeded > (size_t)currentStreamSize) {
        logger->info("Increasing '{}' stream size to {}B", name, totalNeeded);
        streamId = XLinkOpenStream(0, name.c_str(), totalNeeded);
        currentStreamSize = totalNeeded;
    }
}

void XLinkOut::run() {
    using namespace std::chrono;
    using namespace std::chrono_literals;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    std::string streamName = properties.streamName;
    auto tPrevPacket = steady_clock::now();
    auto tPrevSecond = steady_clock::now();
    int packetCounter = 0;

    printf("Opening stream: %s\n", streamName.c_str());

    size_t currentStreamSize = dai::device::XLINK_USB_BUFFER_MAX_SIZE;
    streamId_t streamId = XLinkOpenStream(0, streamName.c_str(), currentStreamSize);

    if(streamId == INVALID_STREAM_ID) {
        logger->error("Cannot open stream (Maximum is {} streams)", XLINK_MAX_STREAMS);
        return;
    }

    std::this_thread::sleep_for(DELAY_AFTER_XLINK_CONNECTION);

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::shared_ptr<ADatatype> message;

        {
            auto blockEvent = this->inputBlockEvent();
            message = input.get<dai::ADatatype>();
        }

        if(!message) continue;
        auto tAfterMessageBeginning = steady_clock::now();

        if(shouldSkipFrame(properties.maxFpsLimit, tPrevPacket, tPrevSecond, packetCounter)) {
            continue;
        }
        if(!message->data) {
            logger->error("XLinkOut no data in message. Skipping this message.");
            continue;
        }

        auto tBeforeSend = steady_clock::now();
        XLinkError_t status = transmitMessage(streamId, message, currentStreamSize, streamName);
        auto tAbsoluteEnd = steady_clock::now();

        if(status == X_LINK_SUCCESS) {
            packetCounter++;
            tPrevPacket = steady_clock::now();

            if(PRINT_DEBUG) {
                printf("PlgXLinkOut | Sent packet, PoBuf size: %d, stream name: %s\n", (int)message->data->getSize(), streamName.c_str());
            }
        } else {
            if(PRINT_DEBUG) printf("PlgXLinkOut | write error, status: %d\n", status);
            std::this_thread::sleep_for(10ms);
        }

        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tAfterMessageBeginning, tBeforeSend, tAbsoluteEnd);
    }
}

}  // namespace gate
}  // namespace dai

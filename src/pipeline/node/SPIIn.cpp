#include "depthai/pipeline/node/SPIIn.hpp"

namespace dai {
namespace node {

void SPIIn::buildInternal() {
    properties.busId = 0;
}

void SPIIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void SPIIn::setBusId(int busId) {
    properties.busId = busId;
}

void SPIIn::setMaxDataSize(std::uint32_t maxDataSize) {
    properties.maxDataSize = maxDataSize;
}

void SPIIn::setNumFrames(std::uint32_t numFrames) {
    properties.numFrames = numFrames;
}

std::string SPIIn::getStreamName() const {
    return properties.streamName;
}

int SPIIn::getBusId() const {
    return properties.busId;
}

std::uint32_t SPIIn::getMaxDataSize() const {
    return properties.maxDataSize;
}

std::uint32_t SPIIn::getNumFrames() const {
    return properties.numFrames;
}

}  // namespace node
}  // namespace dai

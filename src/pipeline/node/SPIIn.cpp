#include "depthai/pipeline/node/SPIIn.hpp"

namespace dai {
namespace node {

SPIIn::SPIIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    properties.busId = 0;
    outputs = {&out};
}

std::string SPIIn::getName() const {
    return "SPIIn";
}

nlohmann::json SPIIn::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> SPIIn::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void SPIIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void SPIIn::setBusId(int id) {
    properties.busId = id;
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

#include "depthai/pipeline/node/XLinkIn.hpp"

namespace dai {
namespace node {

XLinkIn::XLinkIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    outputs = {&out};
}

std::string XLinkIn::getName() const {
    return "XLinkIn";
}

nlohmann::json XLinkIn::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> XLinkIn::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void XLinkIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void XLinkIn::setMaxDataSize(std::uint32_t maxDataSize) {
    properties.maxDataSize = maxDataSize;
}

void XLinkIn::setNumFrames(std::uint32_t numFrames) {
    properties.numFrames = numFrames;
}

std::string XLinkIn::getStreamName() const {
    return properties.streamName;
}

std::uint32_t XLinkIn::getMaxDataSize() const {
    return properties.maxDataSize;
}

std::uint32_t XLinkIn::getNumFrames() const {
    return properties.numFrames;
}

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/UAC.hpp"

#include <cmath>

namespace dai {
namespace node {

UAC::UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string UAC::getName() const {
    return "UAC";
}

std::vector<Node::Input> UAC::getInputs() {
    return {};
}

std::vector<Node::Output> UAC::getOutputs() {
    return {out};
}

nlohmann::json UAC::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> UAC::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void UAC::setStreamBackMic(bool enable) {
    properties.streamBackMic = enable;
}

void UAC::setMicAutoGain(bool enable) {
    properties.enableAgc = enable;
}

void UAC::setMicGainTimes(float times) {
    properties.micGain = times;
}

void UAC::setMicGainDecibels(float dB) {
    float times = pow(10, dB / 20);
    setMicGainTimes(times);
}

void UAC::setXlinkApplyMicGain(bool enable) {
    properties.xLinkApplyMicGain = enable;
}

void UAC::setXlinkSampleSizeBytes(int size) {
    if (size < 2 || size > 4) {
        throw std::runtime_error("UAC | sample size must be: 2, 3 or 4");
    }
    properties.xlinkSampleSizeBytes = size;
}

}  // namespace node
}  // namespace dai

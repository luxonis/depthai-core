#include "depthai/pipeline/node/EdgeDetector.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

EdgeDetector::EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawEdgeDetectorConfig>()), initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&outputImage};
}

std::string EdgeDetector::getName() const {
    return "EdgeDetector";
}

nlohmann::json EdgeDetector::getProperties() {
    nlohmann::json j;
    properties.initialConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

// Node properties configuration
void EdgeDetector::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

void EdgeDetector::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void EdgeDetector::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

std::shared_ptr<Node> EdgeDetector::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai

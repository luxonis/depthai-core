#include "depthai/pipeline/node/EdgeDetector.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

EdgeDetector::EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : EdgeDetector(par, nodeId, std::make_unique<EdgeDetector::Properties>()) {}
EdgeDetector::EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : Node(par, nodeId, std::move(props)),
      properties(static_cast<Properties&>(*Node::properties)),
      rawConfig(std::make_shared<RawEdgeDetectorConfig>()),
      initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&outputImage, &passthroughInputImage};
}

std::string EdgeDetector::getName() const {
    return "EdgeDetector";
}

EdgeDetector::Properties& EdgeDetector::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
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

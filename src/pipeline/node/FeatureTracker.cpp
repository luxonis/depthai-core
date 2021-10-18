#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : FeatureTracker(par, nodeId, std::make_unique<FeatureTracker::Properties>()) {}
FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : Node(par, nodeId, std::move(props)),
      properties(static_cast<Properties&>(*Node::properties)),
      rawConfig(std::make_shared<RawFeatureTrackerConfig>()),
      initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&outputFeatures, &passthroughInputImage};
}

std::string FeatureTracker::getName() const {
    return "FeatureTracker";
}

FeatureTracker::Properties& FeatureTracker::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void FeatureTracker::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

void FeatureTracker::setHardwareResources(int numShaves, int numMemorySlices) {
    properties.numShaves = numShaves;
    properties.numMemorySlices = numMemorySlices;
}

std::shared_ptr<Node> FeatureTracker::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawFeatureTrackerConfig>()), initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&outputFeatures, &passthroughInputImage};
}

std::string FeatureTracker::getName() const {
    return "FeatureTracker";
}

nlohmann::json FeatureTracker::getProperties() {
    nlohmann::json j;
    properties.initialConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
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

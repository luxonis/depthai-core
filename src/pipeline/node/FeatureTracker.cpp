#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawFeatureTrackerConfig>()), initialConfig(rawConfig) {}

std::string FeatureTracker::getName() const {
    return "FeatureTracker";
}

std::vector<Node::Output> FeatureTracker::getOutputs() {
    return {outputFeatures, passthroughInputImage};
}

std::vector<Node::Input> FeatureTracker::getInputs() {
    return {inputConfig, inputImage};
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

std::shared_ptr<Node> FeatureTracker::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai

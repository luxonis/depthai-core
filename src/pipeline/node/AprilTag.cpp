#include "depthai/pipeline/node/AprilTag.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

AprilTag::AprilTag(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawAprilTagConfig>()), initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&out, &passthroughInputImage};
}
std::string AprilTag::getName() const {
    return "AprilTag";
}

nlohmann::json AprilTag::getProperties() {
    nlohmann::json j;
    properties.initialConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

std::shared_ptr<Node> AprilTag::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai

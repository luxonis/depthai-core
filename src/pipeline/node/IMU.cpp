#include "depthai/pipeline/node/IMU.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

IMU::IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string IMU::getName() const {
    return "IMU";
}

std::vector<Node::Output> IMU::getOutputs() {
    return {out};
}

std::vector<Node::Input> IMU::getInputs() {
    return {};
}

nlohmann::json IMU::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> IMU::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

}  // namespace node
}  // namespace dai

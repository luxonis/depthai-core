#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

SystemLogger::SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    properties.rateHz = 1.0f;

    outputs = {&out};
}

std::string SystemLogger::getName() const {
    return "SystemLogger";
}

nlohmann::json SystemLogger::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> SystemLogger::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

}  // namespace node
}  // namespace dai

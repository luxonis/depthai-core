#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

SystemLogger::SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : SystemLogger(par, nodeId, std::make_unique<SystemLogger::Properties>()) {}
SystemLogger::SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : Node(par, nodeId, std::move(props)), properties(static_cast<Properties&>(*Node::properties)) {
    properties.rateHz = 1.0f;

    outputs = {&out};
}

std::string SystemLogger::getName() const {
    return "SystemLogger";
}

SystemLogger::Properties& SystemLogger::getProperties() {
    return properties;
}

std::shared_ptr<Node> SystemLogger::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

}  // namespace node
}  // namespace dai

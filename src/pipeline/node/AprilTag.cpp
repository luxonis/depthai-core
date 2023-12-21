#include "depthai/pipeline/node/AprilTag.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

AprilTag::AprilTag(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(std::move(props)) {}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

}  // namespace node
}  // namespace dai

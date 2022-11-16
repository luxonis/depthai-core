#include "depthai/pipeline/node/AprilTag.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

AprilTag::AprilTag() : NodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(), rawConfig(std::make_shared<RawAprilTagConfig>()), initialConfig(rawConfig) {}

AprilTag::AprilTag(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(std::move(props)),
      rawConfig(std::make_shared<RawAprilTagConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

}  // namespace node
}  // namespace dai

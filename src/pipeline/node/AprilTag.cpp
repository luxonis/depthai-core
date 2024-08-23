#include "depthai/pipeline/node/AprilTag.hpp"
#include <thread>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

AprilTag::AprilTag(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AprilTag, AprilTagProperties>(std::move(props)) {}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

void AprilTag::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool AprilTag::runOnHost() const {
    return runOnHostVar;
}

void AprilTag::run() {
    while(isRunning()) {
        logger->critical("I am running");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/EdgeDetector.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

EdgeDetector::EdgeDetector(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, EdgeDetector, EdgeDetectorProperties>(std::move(props)) {}

EdgeDetector::Properties& EdgeDetector::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

// Node properties configuration
void EdgeDetector::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool EdgeDetector::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void EdgeDetector::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void EdgeDetector::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai

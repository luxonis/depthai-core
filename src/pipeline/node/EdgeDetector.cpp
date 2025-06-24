#include "depthai/pipeline/node/EdgeDetector.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

EdgeDetector::EdgeDetector(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, EdgeDetector, EdgeDetectorProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

EdgeDetector::Properties& EdgeDetector::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void EdgeDetector::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void EdgeDetector::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai

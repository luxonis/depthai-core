#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

FeatureTracker::Properties& FeatureTracker::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void FeatureTracker::setHardwareResources(int numShaves, int numMemorySlices) {
    properties.numShaves = numShaves;
    properties.numMemorySlices = numMemorySlices;
}

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker()
    : NodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties>(), rawConfig(std::make_shared<RawFeatureTrackerConfig>()), initialConfig(rawConfig) {}

FeatureTracker::FeatureTracker(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties>(std::move(props)),
      rawConfig(std::make_shared<RawFeatureTrackerConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

FeatureTracker::Properties& FeatureTracker::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void FeatureTracker::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool FeatureTracker::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void FeatureTracker::setHardwareResources(int numShaves, int numMemorySlices) {
    properties.numShaves = numShaves;
    properties.numMemorySlices = numMemorySlices;
}

}  // namespace node
}  // namespace dai

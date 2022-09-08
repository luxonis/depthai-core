#include "depthai/pipeline/node/FeatureTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties>(par, nodeId, std::make_unique<FeatureTracker::Properties>()),
      rawConfig(std::make_shared<RawFeatureTrackerConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputImage});
    setOutputRefs({&outputFeatures, &passthroughInputImage});
}

FeatureTracker::FeatureTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, FeatureTracker, FeatureTrackerProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawFeatureTrackerConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputImage});
    setOutputRefs({&outputFeatures, &passthroughInputImage});
}

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

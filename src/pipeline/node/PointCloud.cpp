#include "depthai/pipeline/node/PointCloud.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

PointCloud::PointCloud()
    : NodeCRTP<DeviceNode, PointCloud, PointCloudProperties>(), rawConfig(std::make_shared<RawPointCloudConfig>()), initialConfig(rawConfig) {}

PointCloud::PointCloud(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, PointCloud, PointCloudProperties>(std::move(props)),
      rawConfig(std::make_shared<RawPointCloudConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

PointCloud::Properties& PointCloud::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void PointCloud::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool PointCloud::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

}  // namespace node
}  // namespace dai

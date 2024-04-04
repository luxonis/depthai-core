#include "depthai/pipeline/node/PointCloud.hpp"
#include <mutex>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

PointCloud::PointCloud(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>(std::move(props)) {}

PointCloud::Properties& PointCloud::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

void PointCloud::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

}  // namespace node
}  // namespace dai

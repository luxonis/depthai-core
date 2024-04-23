#include "depthai/pipeline/node/PointCloud.hpp"

#include <mutex>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

PointCloud::Properties& PointCloud::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

void PointCloud::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

}  // namespace node
}  // namespace dai

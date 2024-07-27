#include "depthai/pipeline/node/VisionHealth.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

VisionHealth::VisionHealth()
    : NodeCRTP<DeviceNode, VisionHealth, VisionHealthProperties>(), rawConfig(std::make_shared<RawVisionHealthConfig>()), initialConfig(rawConfig) {}

VisionHealth::VisionHealth(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, VisionHealth, VisionHealthProperties>(std::move(props)),
      rawConfig(std::make_shared<RawVisionHealthConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

VisionHealth::Properties& VisionHealth::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

}  // namespace node
}  // namespace dai

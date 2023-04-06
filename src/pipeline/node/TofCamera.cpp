#include "depthai/pipeline/node/TofCamera.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

TofCamera::TofCamera()
    : NodeCRTP<DeviceNode, TofCamera, TofCameraProperties>(), rawConfig(std::make_shared<RawTofCameraConfig>()), initialConfig(rawConfig) {}

TofCamera::TofCamera(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, TofCamera, TofCameraProperties>(std::move(props)),
      rawConfig(std::make_shared<RawTofCameraConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

TofCamera::Properties& TofCamera::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void TofCamera::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool TofCamera::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

}  // namespace node
}  // namespace dai

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

// set raw output size
void TofCamera::setRawSize(int width, int height) {
    properties.rawWidth = width;
    properties.rawHeight = height;
}

int TofCamera::getRawWidth() const {
    return properties.rawWidth;
}

int TofCamera::getRawHeight() const {
    return properties.rawHeight;
}

// set depth output size
void TofCamera::setDepthSize(int width, int height) {
    properties.depthWidth = width;
    properties.depthHeight = height;
}

int TofCamera::getDepthWidth() const {
    return properties.rawWidth;
}

int TofCamera::getDepthHeight() const {
    return properties.depthHeight;
}

dai::TofCameraProperties::TofSensorModel TofCamera::getTofModel() {
    return properties.tofModel;
}

void TofCamera::setTofModel(dai::TofCameraProperties::TofSensorModel model) {
    properties.tofModel = model;
}


}  // namespace node
}  // namespace dai

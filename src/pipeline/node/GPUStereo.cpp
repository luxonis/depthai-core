#include "depthai/pipeline/node/GPUStereo.hpp"

#include <cstdint>
#include <stdexcept>

namespace dai {
namespace node {

GPUStereo::Properties& GPUStereo::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

GPUStereo::GPUStereo(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, GPUStereo, GPUStereoProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

std::shared_ptr<GPUStereo> GPUStereo::build(Output& leftInput, Output& rightInput) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
#endif
    return std::static_pointer_cast<GPUStereo>(shared_from_this());
}

GPUStereo& GPUStereo::setRectification(bool enable) {
    rectificationEnabled = enable;
    properties.rectification = enable;
    return *this;
}

void GPUStereo::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("GPUStereo node is only supported on RVC4 devices.");
        }
    }

    sync->setRunOnHost(false);
    messageDemux->setRunOnHost(false);

    sync->out.link(messageDemux->input);

    // Rectification (if enabled, properties.rectification) runs inside the node on the device's EVA warp engine.
    messageDemux->outputs["left"].link(leftInternal);
    messageDemux->outputs["right"].link(rightInternal);
}

}  // namespace node
}  // namespace dai

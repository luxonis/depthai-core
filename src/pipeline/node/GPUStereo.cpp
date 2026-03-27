#include "depthai/pipeline/node/GPUStereo.hpp"

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
    rectification->enableRectification(enable);
    return *this;
}

void GPUStereo::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("GPUStereo node is not supported on RVC2 devices.");
        }
    }

    sync->out.link(messageDemux->input);
    messageDemux->outputs["left"].link(rectification->input1);
    messageDemux->outputs["right"].link(rectification->input2);
    rectification->output1.link(leftInternal);
    rectification->output2.link(rightInternal);
}

}  // namespace node
}  // namespace dai

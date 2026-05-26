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
    if(rectification) {
        (*rectification)->enableRectification(enable);
    }
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

    if(rectificationEnabled) {
        if(!rectification) rectification = std::make_unique<Subnode<Rectification>>(*this, "rectification");
        (*rectification)->setRunOnHost(false);
        (*rectification)->enableRectification(true);

        messageDemux->outputs["left"].link((*rectification)->input1);
        messageDemux->outputs["right"].link((*rectification)->input2);
        (*rectification)->output1.link(leftInternal);
        (*rectification)->output2.link(rightInternal);
    } else {
        messageDemux->outputs["left"].link(leftInternal);
        messageDemux->outputs["right"].link(rightInternal);
    }
}

}  // namespace node
}  // namespace dai

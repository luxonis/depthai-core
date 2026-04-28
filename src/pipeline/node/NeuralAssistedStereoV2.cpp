#include "depthai/pipeline/node/NeuralAssistedStereoV2.hpp"

namespace dai {
namespace node {

NeuralAssistedStereoV2::Properties& NeuralAssistedStereoV2::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NeuralAssistedStereoV2::NeuralAssistedStereoV2(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralAssistedStereoV2, NeuralAssistedStereoV2Properties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

std::shared_ptr<NeuralAssistedStereoV2> NeuralAssistedStereoV2::build(Output& leftInput, Output& rightInput) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
#endif
    return std::static_pointer_cast<NeuralAssistedStereoV2>(shared_from_this());
}

NeuralAssistedStereoV2& NeuralAssistedStereoV2::setRectification(bool enable) {
    rectification->enableRectification(enable);
    return *this;
}

void NeuralAssistedStereoV2::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("NeuralAssistedStereoV2 node is not supported on RVC2 devices.");
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

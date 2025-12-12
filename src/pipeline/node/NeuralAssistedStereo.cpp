#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

namespace dai {
namespace node {

NeuralAssistedStereo::Properties& NeuralAssistedStereo::getProperties() {
    return properties;
}

NeuralAssistedStereo::~NeuralAssistedStereo() = default;

NeuralAssistedStereo::NeuralAssistedStereo(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralAssistedStereo, NeuralAssistedStereoProperties>(std::move(props)) {}

NeuralAssistedStereo::NeuralAssistedStereo()
    : DeviceNodeCRTP<DeviceNode, NeuralAssistedStereo, NeuralAssistedStereoProperties>(std::make_unique<Properties>()) {}

void NeuralAssistedStereo::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("NeuralAssistedStereo node is not supported on RVC2 devices.");
        }
    }
}

std::shared_ptr<NeuralAssistedStereo> NeuralAssistedStereo::build(Output& leftInput, Output& rightInput, DeviceModelZoo neuralModel, bool rectifyImages) {
    // Link camera inputs to rectification
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
#endif

    if(rectifyImages) {
        neuralDepth->build(rectification->output1, rectification->output2, neuralModel);
        vpp->build(rectification->output1, rectification->output2, neuralDepth->disparity, neuralDepth->confidence);
    } else {
        vpp->build(leftInput, rightInput, neuralDepth->disparity, neuralDepth->confidence);
    }

    neuralDepth->setRectification(false);
    stereoDepth->setRectification(false);

    vpp->leftOut.link(stereoDepth->left);
    vpp->rightOut.link(stereoDepth->right);

    return std::static_pointer_cast<NeuralAssistedStereo>(shared_from_this());
}

}  // namespace node
}  // namespace dai

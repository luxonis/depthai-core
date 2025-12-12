#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

namespace dai {
namespace node {



NeuralAssistedStereo::Properties& NeuralAssistedStereo::getProperties() {
    properties.vppConfig = *vppConfig;
    properties.stereoConfig = *stereoConfig;
    properties.neuralConfig = *neuralConfig;
    return properties;
}

NeuralAssistedStereo::NeuralAssistedStereo(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralAssistedStereo, NeuralAssistedStereoProperties>(std::move(props)),
      vppConfig(std::make_shared<decltype(properties.vppConfig)>(properties.vppConfig)),
      stereoConfig(std::make_shared<decltype(properties.stereoConfig)>(properties.stereoConfig)),
      neuralConfig(std::make_shared<decltype(properties.neuralConfig)>(properties.neuralConfig)) {}

bool NeuralAssistedStereo::runOnHost() const {
    // Composite node is host-side only; only subnodes are serialized to device
    return true;
}

std::shared_ptr<NeuralAssistedStereo> NeuralAssistedStereo::build(Output& leftInput, Output& rightInput, DeviceModelZoo neuralModel) {
    // Non-rectified camera inputs go to TWO places:
    
    // 1. To Rectification node (full resolution)
    leftInput.link(left);    // left = rectification->input1
    rightInput.link(right);  // right = rectification->input2
    
    // 2. To NeuralDepth (has its own internal rectification)
    neuralDepth->build(leftInput, rightInput, neuralModel);
    
    return std::static_pointer_cast<NeuralAssistedStereo>(shared_from_this());
}




void NeuralAssistedStereo::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("NeuralAssistedStereo node is not supported on RVC2 devices.");
        }
    }

    // Initialize configs with default values
    *vpp->initialConfig = *vppConfig;
    *stereoDepth->initialConfig = *stereoConfig;
    *neuralDepth->initialConfig = *neuralConfig;

    // INTERNAL SUBNODE CONNECTIONS:
    // Rectification (full res) → VPP
    rectification->output1.link(*vpp->left);
    rectification->output2.link(*vpp->right);

    // NeuralDepth outputs → VPP
    neuralDepth->disparity.link(*vpp->disparity);
    neuralDepth->confidence.link(*vpp->confidence);

    // VPP outputs → StereoDepth
    vpp->leftOut.link(stereoDepth->left);
    vpp->rightOut.link(stereoDepth->right);

    // StereoDepth should not rectify (already done by VPP)
    stereoDepth->setRectification(false);
}

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

namespace dai {
namespace node {

NeuralAssistedStereo::Properties& NeuralAssistedStereo::getProperties() {
    properties.vppConfig = *vppConfig;
    properties.stereoConfig = *stereoConfig;
    return properties;
}

NeuralAssistedStereo::NeuralAssistedStereo(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralAssistedStereo, NeuralAssistedStereoProperties>(std::move(props)),
      vppConfig(std::make_shared<decltype(properties.vppConfig)>(properties.vppConfig)),
      stereoConfig(std::make_shared<decltype(properties.stereoConfig)>(properties.stereoConfig)) {}

std::shared_ptr<NeuralAssistedStereo> NeuralAssistedStereo::build(Output& leftInput, Output& rightInput, DeviceModelZoo neuralModel) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    // Link camera inputs to rectification
    leftInput.link(left);
    rightInput.link(right);
    
    // Build neural depth with the same inputs (it has its own internal rectification)
    neuralDepth->build(leftInput, rightInput, neuralModel);
    
    // Link rectification outputs (full res) to VPP
    rectification->output1.link(*vpp->left);
    rectification->output2.link(*vpp->right);
    
    // Link neural depth outputs to VPP
    neuralDepth->disparity.link(*vpp->disparity);
    neuralDepth->confidence.link(*vpp->confidence);
    
    // Link VPP outputs to stereo depth
    vpp->leftOut.link(stereoDepth->left);
    vpp->rightOut.link(stereoDepth->right);
    
    // Stereo should not rectify again (already done)
    stereoDepth->setRectification(false);
#endif
    
    return std::static_pointer_cast<NeuralAssistedStereo>(shared_from_this());
}

NeuralAssistedStereo& NeuralAssistedStereo::setStereoRectification(bool enable) {
    stereoDepth->setRectification(enable);
    return *this;
}

NeuralAssistedStereo& NeuralAssistedStereo::setExtendedDisparity(bool enable) {
    stereoDepth->setExtendedDisparity(enable);
    return *this;
}

NeuralAssistedStereo& NeuralAssistedStereo::setSubpixel(bool enable) {
    stereoDepth->setSubpixel(enable);
    return *this;
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
}

}  // namespace node
}  // namespace dai

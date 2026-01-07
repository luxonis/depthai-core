#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

namespace dai {
namespace node {

NeuralAssistedStereo::NeuralAssistedStereo(const std::shared_ptr<Device>& device) : DeviceNodeGroup(device) {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("NeuralAssistedStereo node is not supported on RVC2 devices.");
        }
    }
    setInitialValues();
}

NeuralAssistedStereo::~NeuralAssistedStereo() = default;

void NeuralAssistedStereo::setInitialValues() {
    // vpp parameters
    vpp->initialConfig->blending = 0.3;
    vpp->initialConfig->maxPatchSize = 4;
    vpp->initialConfig->uniformPatch = false;
    vpp->initialConfig->injectionParameters.textureThreshold = 13493.682329122857;
    vpp->initialConfig->injectionParameters.kernelSize = 9;
    vpp->initialConfig->injectionParameters.useInjection = true;
    vpp->initialConfig->maxNumThreads = 8;

    neuralDepth->initialConfig->setConfidenceThreshold(static_cast<int>(0.14 * 255));

    // // Stereo node (fixed config for now)
    stereoDepth->setRectification(false);
    stereoDepth->setExtendedDisparity(true);
    stereoDepth->setLeftRightCheck(true);
    stereoDepth->setSubpixel(true);

    stereoDepth->initialConfig->setConfidenceThreshold(250);
    stereoDepth->initialConfig->setLeftRightCheckThreshold(3);
    stereoDepth->initialConfig->setMedianFilter(dai::filters::params::MedianFilter::MEDIAN_OFF);

    stereoDepth->initialConfig->algorithmControl.enableSwLeftRightCheck = false;
    stereoDepth->initialConfig->algorithmControl.numInvalidateEdgePixels = 100;

    stereoDepth->initialConfig->censusTransform.noiseThresholdOffset = 0;
    stereoDepth->initialConfig->censusTransform.noiseThresholdScale = -96;

    stereoDepth->initialConfig->confidenceMetrics.flatnessConfidenceThreshold = 4;
    stereoDepth->initialConfig->confidenceMetrics.flatnessConfidenceWeight = 2;
    stereoDepth->initialConfig->confidenceMetrics.motionVectorConfidenceWeight = 16;
    stereoDepth->initialConfig->confidenceMetrics.occlusionConfidenceWeight = 14;

    stereoDepth->initialConfig->costAggregation.p1Config.defaultValue = 16;
    stereoDepth->initialConfig->costAggregation.p1Config.enableAdaptive = false;

    stereoDepth->initialConfig->costAggregation.p2Config.defaultValue = 65;
    stereoDepth->initialConfig->costAggregation.p2Config.enableAdaptive = false;

    stereoDepth->initialConfig->costMatching.enableSwConfidenceThresholding = false;

    stereoDepth->initialConfig->postProcessing.adaptiveMedianFilter.enable = true;
    stereoDepth->initialConfig->postProcessing.brightnessFilter.maxBrightness = 255;
    stereoDepth->initialConfig->postProcessing.brightnessFilter.minBrightness = 0;
    stereoDepth->initialConfig->postProcessing.decimationFilter.decimationFactor = 1;
    stereoDepth->initialConfig->postProcessing.decimationFilter.decimationMode =
        dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;
    stereoDepth->initialConfig->postProcessing.holeFilling.enable = true;
    stereoDepth->initialConfig->postProcessing.holeFilling.fillConfidenceThreshold = 245;
    stereoDepth->initialConfig->postProcessing.holeFilling.highConfidenceThreshold = 235;
    stereoDepth->initialConfig->postProcessing.holeFilling.minValidDisparity = 1;
    stereoDepth->initialConfig->postProcessing.spatialFilter.enable = false;
    stereoDepth->initialConfig->postProcessing.speckleFilter.enable = true;
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

#include "depthai/pipeline/node/NeuralDepth.hpp"

#include <fmt/format.h>  // fmt::format
#include <fmt/std.h>     // std::filesystem::path formatting

#include <fstream>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/pipeline/node/Camera.hpp"

namespace dai {
namespace node {

std::shared_ptr<NeuralDepth> NeuralDepth::build(bool autoCreateCameras, PresetMode presetMode, std::pair<int, int> size, std::optional<float> fps) {
    if(!autoCreateCameras) {
        return std::static_pointer_cast<NeuralDepth>(shared_from_this());
    }
    // TODO(Morato) - push this further, consider if cameras have already been used etc.
    // First get the default stereo pairs
    auto stereoPairs = device->getAvailableStereoPairs();
    if(stereoPairs.empty()) {
        auto deviceName = device->getDeviceName();
        auto boardName = device->readCalibration().getEepromData().boardName;
        throw std::runtime_error(fmt::format("Device {} ({}) does not have stereo pair available", deviceName, boardName));
    }
    // Take the first stereo pair
    auto stereoPair = stereoPairs[0];
    // Create the two cameras
    auto pipeline = getParentPipeline();
    auto left = pipeline.create<dai::node::Camera>()->build(stereoPair.left);
    auto right = pipeline.create<dai::node::Camera>()->build(stereoPair.right);

    return build(
        *left->requestOutput(size, std::nullopt, ImgResizeMode::CROP, fps), *right->requestOutput(size, std::nullopt, ImgResizeMode::CROP, fps), presetMode);
}

NeuralDepth::NeuralDepth(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralDepth, NeuralDepthProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

NeuralDepth::Properties& NeuralDepth::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void NeuralDepth::loadMeshData(const std::vector<std::uint8_t>& dataLeft, const std::vector<std::uint8_t>& dataRight) {
    if(dataLeft.size() != dataRight.size()) {
        throw std::runtime_error("NeuralDepth | left and right mesh sizes must match");
    }

    Asset meshAsset;
    std::string assetKey;
    meshAsset.alignment = 64;

    meshAsset.data = dataLeft;
    assetKey = "meshLeft";
    properties.mesh.meshLeftUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();

    meshAsset.data = dataRight;
    assetKey = "meshRight";
    properties.mesh.meshRightUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();

    properties.mesh.meshSize = static_cast<uint32_t>(meshAsset.data.size());
}

void NeuralDepth::loadMeshFiles(const std::filesystem::path& pathLeft, const std::filesystem::path& pathRight) {
    std::ifstream streamLeft(pathLeft, std::ios::binary);
    if(!streamLeft.is_open()) {
        throw std::runtime_error(fmt::format("NeuralDepth | Cannot open mesh at path: {}", pathLeft));
    }
    std::vector<std::uint8_t> dataLeft = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamLeft), {});

    std::ifstream streamRight(pathRight, std::ios::binary);
    if(!streamRight.is_open()) {
        throw std::runtime_error(fmt::format("NeuralDepth | Cannot open mesh at path: {}", pathRight));
    }
    std::vector<std::uint8_t> dataRight = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamRight), {});

    loadMeshData(dataLeft, dataRight);
}

void NeuralDepth::setMeshStep(int width, int height) {
    properties.mesh.stepWidth = width;
    properties.mesh.stepHeight = height;
}

void NeuralDepth::setInputResolution(int width, int height) {
    properties.width = width;
    properties.height = height;
}
void NeuralDepth::setInputResolution(std::tuple<int, int> resolution) {
    setInputResolution(std::get<0>(resolution), std::get<1>(resolution));
}
void NeuralDepth::setOutputSize(int width, int height) {
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC4) {
            throw std::runtime_error("NeuralDepth | setOutputSize is not supported on RVC4 platform");
        }
    }
    properties.outWidth = width;
    properties.outHeight = height;
}
void NeuralDepth::setOutputKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
}

void NeuralDepth::setDepthAlign(Properties::DepthAlign align) {
    initialConfig->setDepthAlign(align);
    // Unset 'depthAlignCamera', that would take precedence otherwise
    properties.depthAlignCamera = CameraBoardSocket::AUTO;
}
void NeuralDepth::setDepthAlign(CameraBoardSocket camera) {
    properties.depthAlignCamera = camera;
}

void NeuralDepth::setRectification(bool enable) {
    properties.enableRectification = enable;
}
void NeuralDepth::setLeftRightCheck(bool enable) {
    initialConfig->setLeftRightCheck(enable);
    properties.initialConfig = *initialConfig;
}
void NeuralDepth::setSubpixel(bool enable) {
    initialConfig->setSubpixel(enable);
    properties.initialConfig = *initialConfig;
}
void NeuralDepth::setSubpixelFractionalBits(int subpixelFractionalBits) {
    initialConfig->setSubpixelFractionalBits(subpixelFractionalBits);
    properties.initialConfig = *initialConfig;
}
void NeuralDepth::setExtendedDisparity(bool enable) {
    initialConfig->setExtendedDisparity(enable);
    properties.initialConfig = *initialConfig;
}
void NeuralDepth::setRectifyEdgeFillColor(int color) {
    properties.rectifyEdgeFillColor = color;
}

void NeuralDepth::setRuntimeModeSwitch(bool enable) {
    properties.enableRuntimeStereoModeSwitch = enable;
}

void NeuralDepth::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void NeuralDepth::setPostProcessingHardwareResources(int numShaves, int numMemorySlices) {
    properties.numPostProcessingShaves = numShaves;
    properties.numPostProcessingMemorySlices = numMemorySlices;
}

void NeuralDepth::useHomographyRectification(bool useHomographyRectification) {
    properties.useHomographyRectification = useHomographyRectification;
}

void NeuralDepth::enableDistortionCorrection(bool enableDistortionCorrection) {
    useHomographyRectification(!enableDistortionCorrection);
}

void NeuralDepth::setBaseline(float baseline) {
    properties.baseline = baseline;
}

void NeuralDepth::setFocalLength(float focalLength) {
    properties.focalLength = focalLength;
}

void NeuralDepth::setDisparityToDepthUseSpecTranslation(bool specTranslation) {
    properties.disparityToDepthUseSpecTranslation = specTranslation;
}

void NeuralDepth::setRectificationUseSpecTranslation(bool specTranslation) {
    properties.rectificationUseSpecTranslation = specTranslation;
}

void NeuralDepth::setDepthAlignmentUseSpecTranslation(bool specTranslation) {
    properties.depthAlignmentUseSpecTranslation = specTranslation;
}

void NeuralDepth::setAlphaScaling(float alpha) {
    properties.alphaScaling = alpha;
}

void NeuralDepth::setDefaultProfilePreset(PresetMode mode) {
    presetMode = mode;

    switch(presetMode) {
        case PresetMode::FAST_ACCURACY: {
            initialConfig->setConfidenceThreshold(55);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setLeftRightCheckThreshold(5);

            initialConfig->postProcessing.holeFilling.enable = true;
            initialConfig->postProcessing.adaptiveMedianFilter.enable = true;

            initialConfig->confidenceMetrics.occlusionConfidenceWeight = 20;
            initialConfig->confidenceMetrics.motionVectorConfidenceWeight = 4;
            initialConfig->confidenceMetrics.flatnessConfidenceWeight = 4;
            initialConfig->confidenceMetrics.flatnessConfidenceThreshold = 2;

            initialConfig->costAggregation.p1Config.defaultValue = 11;
            initialConfig->costAggregation.p1Config.edgeValue = 10;
            initialConfig->costAggregation.p1Config.smoothValue = 22;

        } break;
        case PresetMode::FAST_DENSITY: {
            initialConfig->setConfidenceThreshold(15);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setLeftRightCheckThreshold(10);

            initialConfig->postProcessing.holeFilling.enable = true;
            initialConfig->postProcessing.holeFilling.highConfidenceThreshold = 100;
            initialConfig->postProcessing.holeFilling.fillConfidenceThreshold = 210;
            initialConfig->postProcessing.holeFilling.minValidDisparity = 3;

            initialConfig->postProcessing.adaptiveMedianFilter.enable = true;

            initialConfig->confidenceMetrics.occlusionConfidenceWeight = 20;
            initialConfig->confidenceMetrics.motionVectorConfidenceWeight = 10;
            initialConfig->confidenceMetrics.flatnessConfidenceWeight = 2;
            initialConfig->confidenceMetrics.flatnessConfidenceThreshold = 5;

            initialConfig->costAggregation.p1Config.defaultValue = 45;
            initialConfig->costAggregation.p1Config.edgeValue = 40;
            initialConfig->costAggregation.p1Config.smoothValue = 49;

            initialConfig->costAggregation.p2Config.defaultValue = 95;
            initialConfig->costAggregation.p2Config.edgeValue = 90;
            initialConfig->costAggregation.p2Config.smoothValue = 99;
        } break;
        case PresetMode::DEFAULT: {
            setDefaultProfilePreset(PresetMode::FAST_DENSITY);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setExtendedDisparity(false);
            initialConfig->setSubpixel(true);
            initialConfig->setSubpixelFractionalBits(3);
            initialConfig->setMedianFilter(MedianFilter::KERNEL_7x7);

            initialConfig->postProcessing.filteringOrder = {NeuralDepthConfig::PostProcessing::Filter::DECIMATION,
                                                            NeuralDepthConfig::PostProcessing::Filter::MEDIAN,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPECKLE,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPATIAL,
                                                            NeuralDepthConfig::PostProcessing::Filter::TEMPORAL};
            initialConfig->postProcessing.decimationFilter.decimationFactor = 2;
            initialConfig->postProcessing.decimationFilter.decimationMode = NeuralDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;

            initialConfig->postProcessing.spatialFilter.enable = true;
            initialConfig->postProcessing.spatialFilter.holeFillingRadius = 1;
            initialConfig->postProcessing.spatialFilter.numIterations = 1;
            initialConfig->postProcessing.spatialFilter.alpha = 0.5;
            initialConfig->postProcessing.spatialFilter.delta = 3;

            initialConfig->postProcessing.temporalFilter.enable = true;
            initialConfig->postProcessing.temporalFilter.alpha = 0.5;
            initialConfig->postProcessing.temporalFilter.delta = 3;

            initialConfig->postProcessing.speckleFilter.enable = true;
            initialConfig->postProcessing.speckleFilter.speckleRange = 200;
            initialConfig->postProcessing.speckleFilter.differenceThreshold = 2;

            initialConfig->postProcessing.thresholdFilter.minRange = 0;
            initialConfig->postProcessing.thresholdFilter.maxRange = 15000;

            setPostProcessingHardwareResources(3, 3);
        } break;
        case PresetMode::FACE: {
            setDefaultProfilePreset(PresetMode::FAST_DENSITY);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setExtendedDisparity(true);
            initialConfig->setSubpixel(true);
            initialConfig->setSubpixelFractionalBits(5);
            initialConfig->setMedianFilter(MedianFilter::MEDIAN_OFF);

            initialConfig->postProcessing.filteringOrder = {NeuralDepthConfig::PostProcessing::Filter::DECIMATION,
                                                            NeuralDepthConfig::PostProcessing::Filter::MEDIAN,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPECKLE,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPATIAL,
                                                            NeuralDepthConfig::PostProcessing::Filter::TEMPORAL};
            initialConfig->postProcessing.decimationFilter.decimationFactor = 2;
            initialConfig->postProcessing.decimationFilter.decimationMode = NeuralDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;

            initialConfig->postProcessing.spatialFilter.enable = true;
            initialConfig->postProcessing.spatialFilter.holeFillingRadius = 1;
            initialConfig->postProcessing.spatialFilter.numIterations = 1;
            initialConfig->postProcessing.spatialFilter.alpha = 0.5;
            initialConfig->postProcessing.spatialFilter.delta = 3;

            initialConfig->postProcessing.temporalFilter.enable = true;
            initialConfig->postProcessing.temporalFilter.alpha = 0.5;
            initialConfig->postProcessing.temporalFilter.delta = 3;

            initialConfig->postProcessing.speckleFilter.enable = true;
            initialConfig->postProcessing.speckleFilter.speckleRange = 200;
            initialConfig->postProcessing.speckleFilter.differenceThreshold = 2;

            initialConfig->postProcessing.thresholdFilter.minRange = 30;
            initialConfig->postProcessing.thresholdFilter.maxRange = 3000;

            setPostProcessingHardwareResources(3, 3);
        } break;
        case PresetMode::HIGH_DETAIL: {
            setDefaultProfilePreset(PresetMode::FAST_ACCURACY);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setExtendedDisparity(true);
            initialConfig->setSubpixel(true);
            initialConfig->setSubpixelFractionalBits(5);
            initialConfig->setMedianFilter(MedianFilter::MEDIAN_OFF);

            initialConfig->postProcessing.filteringOrder = {NeuralDepthConfig::PostProcessing::Filter::DECIMATION,
                                                            NeuralDepthConfig::PostProcessing::Filter::MEDIAN,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPECKLE,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPATIAL,
                                                            NeuralDepthConfig::PostProcessing::Filter::TEMPORAL};
            initialConfig->postProcessing.decimationFilter.decimationFactor = 2;
            initialConfig->postProcessing.decimationFilter.decimationMode = NeuralDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;

            initialConfig->postProcessing.spatialFilter.enable = true;
            initialConfig->postProcessing.spatialFilter.holeFillingRadius = 1;
            initialConfig->postProcessing.spatialFilter.numIterations = 1;
            initialConfig->postProcessing.spatialFilter.alpha = 0.5;
            initialConfig->postProcessing.spatialFilter.delta = 3;

            initialConfig->postProcessing.temporalFilter.enable = true;
            initialConfig->postProcessing.temporalFilter.alpha = 0.5;
            initialConfig->postProcessing.temporalFilter.delta = 3;

            initialConfig->postProcessing.speckleFilter.enable = true;
            initialConfig->postProcessing.speckleFilter.speckleRange = 200;
            initialConfig->postProcessing.speckleFilter.differenceThreshold = 2;

            initialConfig->postProcessing.thresholdFilter.minRange = 0;
            initialConfig->postProcessing.thresholdFilter.maxRange = 15000;

            setPostProcessingHardwareResources(3, 3);
        } break;
        case PresetMode::ROBOTICS: {
            setDefaultProfilePreset(PresetMode::FAST_DENSITY);
            initialConfig->setLeftRightCheck(true);
            initialConfig->setExtendedDisparity(false);
            initialConfig->setSubpixel(true);
            initialConfig->setSubpixelFractionalBits(3);
            initialConfig->setMedianFilter(MedianFilter::KERNEL_7x7);

            initialConfig->postProcessing.filteringOrder = {NeuralDepthConfig::PostProcessing::Filter::DECIMATION,
                                                            NeuralDepthConfig::PostProcessing::Filter::MEDIAN,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPECKLE,
                                                            NeuralDepthConfig::PostProcessing::Filter::SPATIAL,
                                                            NeuralDepthConfig::PostProcessing::Filter::TEMPORAL};
            initialConfig->postProcessing.decimationFilter.decimationFactor = 2;
            initialConfig->postProcessing.decimationFilter.decimationMode = NeuralDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;

            initialConfig->postProcessing.spatialFilter.enable = true;
            initialConfig->postProcessing.spatialFilter.holeFillingRadius = 2;
            initialConfig->postProcessing.spatialFilter.numIterations = 1;
            initialConfig->postProcessing.spatialFilter.alpha = 0.5;
            initialConfig->postProcessing.spatialFilter.delta = 20;

            initialConfig->postProcessing.temporalFilter.enable = false;
            initialConfig->postProcessing.temporalFilter.alpha = 0.5;
            initialConfig->postProcessing.temporalFilter.delta = 3;

            initialConfig->postProcessing.speckleFilter.enable = true;
            initialConfig->postProcessing.speckleFilter.speckleRange = 200;
            initialConfig->postProcessing.speckleFilter.differenceThreshold = 2;

            initialConfig->postProcessing.thresholdFilter.minRange = 0;
            initialConfig->postProcessing.thresholdFilter.maxRange = 10000;

            setPostProcessingHardwareResources(3, 3);
        } break;
    }
}

void NeuralDepth::setFrameSync(bool enableFrameSync) {
    properties.enableFrameSync = enableFrameSync;
}

}  // namespace node
}  // namespace dai

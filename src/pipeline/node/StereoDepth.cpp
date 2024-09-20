#include "depthai/pipeline/node/StereoDepth.hpp"

#include "depthai/pipeline/node/MonoCamera.hpp"
// standard
#include <fstream>
#include <memory>

#include "pipeline/datatype/StereoDepthConfig.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

namespace dai {
namespace node {

std::shared_ptr<StereoDepth> StereoDepth::build(bool autoCreateCameras, PresetMode presetMode) {
    if(!autoCreateCameras) {
        return std::static_pointer_cast<StereoDepth>(shared_from_this());
    }
    // TODO(Morato) - push this further, consider if cameras have already been used etc.
    // First get the default stereo pairs
    auto stereoPairs = device->getAvailableStereoPairs();
    if(stereoPairs.empty()) {
        throw std::runtime_error("No stereo pairs available, can't auto-create StereoDepth node");
    }
    // Take the first stereo pair
    auto stereoPair = stereoPairs[0];
    // Create the two cameras
    auto pipeline = getParentPipeline();
    auto left = pipeline.create<dai::node::MonoCamera>();
    left->setBoardSocket(stereoPair.left);
    auto right = pipeline.create<dai::node::MonoCamera>();
    right->setBoardSocket(stereoPair.right);

    return build(left->out, right->out, presetMode);
}

StereoDepth::StereoDepth(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, StereoDepth, StereoDepthProperties>(std::move(props)), initialConfig(properties.initialConfig) {}

StereoDepth::Properties& StereoDepth::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

void StereoDepth::setEmptyCalibration(void) {
    setRectification(false);
    logger::warn("{} is deprecated. This function call can be replaced by Stereo::setRectification(false). ", __func__);
}

void StereoDepth::loadMeshData(const std::vector<std::uint8_t>& dataLeft, const std::vector<std::uint8_t>& dataRight) {
    if(dataLeft.size() != dataRight.size()) {
        throw std::runtime_error("StereoDepth | left and right mesh sizes must match");
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

void StereoDepth::loadMeshFiles(const dai::Path& pathLeft, const dai::Path& pathRight) {
    std::ifstream streamLeft(pathLeft, std::ios::binary);
    if(!streamLeft.is_open()) {
        throw std::runtime_error(fmt::format("StereoDepth | Cannot open mesh at path: {}", pathLeft));
    }
    std::vector<std::uint8_t> dataLeft = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamLeft), {});

    std::ifstream streamRight(pathRight, std::ios::binary);
    if(!streamRight.is_open()) {
        throw std::runtime_error(fmt::format("StereoDepth | Cannot open mesh at path: {}", pathRight));
    }
    std::vector<std::uint8_t> dataRight = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamRight), {});

    loadMeshData(dataLeft, dataRight);
}

void StereoDepth::setMeshStep(int width, int height) {
    properties.mesh.stepWidth = width;
    properties.mesh.stepHeight = height;
}

void StereoDepth::setInputResolution(int width, int height) {
    properties.width = width;
    properties.height = height;
}
void StereoDepth::setInputResolution(std::tuple<int, int> resolution) {
    setInputResolution(std::get<0>(resolution), std::get<1>(resolution));
}
void StereoDepth::setOutputSize(int width, int height) {
    properties.outWidth = width;
    properties.outHeight = height;
}
void StereoDepth::setOutputKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
}
void StereoDepth::setMedianFilter(dai::StereoDepthConfig::MedianFilter median) {
    initialConfig.setMedianFilter(median);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setDepthAlign(Properties::DepthAlign align) {
    initialConfig.setDepthAlign(align);
    // Unset 'depthAlignCamera', that would take precedence otherwise
    properties.depthAlignCamera = CameraBoardSocket::AUTO;
}
void StereoDepth::setDepthAlign(CameraBoardSocket camera) {
    properties.depthAlignCamera = camera;
}
void StereoDepth::setConfidenceThreshold(int confThr) {
    initialConfig.setConfidenceThreshold(confThr);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setRectification(bool enable) {
    properties.enableRectification = enable;
}
void StereoDepth::setLeftRightCheck(bool enable) {
    initialConfig.setLeftRightCheck(enable);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setSubpixel(bool enable) {
    initialConfig.setSubpixel(enable);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setSubpixelFractionalBits(int subpixelFractionalBits) {
    initialConfig.setSubpixelFractionalBits(subpixelFractionalBits);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setExtendedDisparity(bool enable) {
    initialConfig.setExtendedDisparity(enable);
    properties.initialConfig = initialConfig;
}
void StereoDepth::setRectifyEdgeFillColor(int color) {
    properties.rectifyEdgeFillColor = color;
}
void StereoDepth::setRectifyMirrorFrame(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated.", __func__);
}
void StereoDepth::setOutputRectified(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}
void StereoDepth::setOutputDepth(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}

void StereoDepth::setRuntimeModeSwitch(bool enable) {
    properties.enableRuntimeStereoModeSwitch = enable;
}

void StereoDepth::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

float StereoDepth::getMaxDisparity() const {
    return initialConfig.getMaxDisparity();
}

void StereoDepth::setPostProcessingHardwareResources(int numShaves, int numMemorySlices) {
    properties.numPostProcessingShaves = numShaves;
    properties.numPostProcessingMemorySlices = numMemorySlices;
}

void StereoDepth::setFocalLengthFromCalibration(bool focalLengthFromCalibration) {
    properties.focalLengthFromCalibration = focalLengthFromCalibration;
}

void StereoDepth::useHomographyRectification(bool useHomographyRectification) {
    properties.useHomographyRectification = useHomographyRectification;
}

void StereoDepth::enableDistortionCorrection(bool enableDistortionCorrection) {
    useHomographyRectification(!enableDistortionCorrection);
}

void StereoDepth::setVerticalStereo(bool verticalStereo) {
    properties.verticalStereo = verticalStereo;
}

void StereoDepth::setCustomPixelDescriptors(bool customPixelDescriptors) {
    properties.customPixelDescriptors = customPixelDescriptors;
}

void StereoDepth::setBaseline(float baseline) {
    properties.baseline = baseline;
}

void StereoDepth::setFocalLength(float focalLength) {
    properties.focalLength = focalLength;
}

void StereoDepth::setDisparityToDepthUseSpecTranslation(bool specTranslation) {
    properties.disparityToDepthUseSpecTranslation = specTranslation;
}

void StereoDepth::setRectificationUseSpecTranslation(bool specTranslation) {
    properties.rectificationUseSpecTranslation = specTranslation;
}

void StereoDepth::setDepthAlignmentUseSpecTranslation(bool specTranslation) {
    properties.depthAlignmentUseSpecTranslation = specTranslation;
}

void StereoDepth::setAlphaScaling(float alpha) {
    properties.alphaScaling = alpha;
}

void StereoDepth::setDefaultProfilePreset(PresetMode mode) {
    presetMode = mode;
    switch(presetMode) {
        case PresetMode::HIGH_ACCURACY: {
            initialConfig.setConfidenceThreshold(55);
            initialConfig.setLeftRightCheck(true);
            initialConfig.setLeftRightCheckThreshold(5);

            initialConfig.postProcessing.holeFilling.enable = true;
            initialConfig.postProcessing.adaptiveMedianFilter.enable = true;

            initialConfig.confidenceMetrics.occlusionConfidenceWeight = 20;
            initialConfig.confidenceMetrics.motionVectorConfidenceWeight = 4;
            initialConfig.confidenceMetrics.flatnessConfidenceWeight = 4;
            initialConfig.confidenceMetrics.flatnessConfidenceThreshold = 2;

            initialConfig.costAggregation.p1Config.defaultValue = 11;
            initialConfig.costAggregation.p1Config.edgeValue = 10;
            initialConfig.costAggregation.p1Config.smoothValue = 22;

        } break;
        case PresetMode::HIGH_DENSITY: {
            initialConfig.setConfidenceThreshold(15);
            initialConfig.setLeftRightCheck(true);
            initialConfig.setLeftRightCheckThreshold(10);

            initialConfig.postProcessing.holeFilling.enable = true;
            initialConfig.postProcessing.holeFilling.highConfidenceThreshold = 100;
            initialConfig.postProcessing.holeFilling.fillConfidenceThreshold = 210;
            initialConfig.postProcessing.holeFilling.minValidDisparity = 3;

            initialConfig.postProcessing.adaptiveMedianFilter.enable = true;

            initialConfig.confidenceMetrics.occlusionConfidenceWeight = 20;
            initialConfig.confidenceMetrics.motionVectorConfidenceWeight = 10;
            initialConfig.confidenceMetrics.flatnessConfidenceWeight = 2;
            initialConfig.confidenceMetrics.flatnessConfidenceThreshold = 5;

            initialConfig.costAggregation.p1Config.defaultValue = 45;
            initialConfig.costAggregation.p1Config.edgeValue = 40;
            initialConfig.costAggregation.p1Config.smoothValue = 49;

            initialConfig.costAggregation.p2Config.defaultValue = 95;
            initialConfig.costAggregation.p2Config.edgeValue = 90;
            initialConfig.costAggregation.p2Config.smoothValue = 99;
        } break;
    }
}

void StereoDepth::setFrameSync(bool enableFrameSync) {
    properties.enableFrameSync = enableFrameSync;
}

}  // namespace node
}  // namespace dai

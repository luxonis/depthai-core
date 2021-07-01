#include "depthai/pipeline/node/StereoDepth.hpp"

// standard
#include <fstream>

#include "spdlog/spdlog.h"

namespace dai {
namespace node {

StereoDepth::StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawStereoDepthConfig>()), initialConfig(rawConfig) {
    // 'properties' defaults already set
}

std::string StereoDepth::getName() const {
    return "StereoDepth";
}

std::vector<Node::Output> StereoDepth::getOutputs() {
    return {disparity, depth, syncedLeft, syncedRight, rectifiedLeft, rectifiedRight};
}

std::vector<Node::Input> StereoDepth::getInputs() {
    return {inputConfig, left, right};
}

nlohmann::json StereoDepth::getProperties() {
    nlohmann::json j;
    properties.initialConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> StereoDepth::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void StereoDepth::loadCalibrationData(const std::vector<std::uint8_t>& data) {
    (void)data;
    spdlog::warn("{} is deprecated. This function call is replaced by Pipeline::setCalibrationData under pipeline. ", __func__);
}

void StereoDepth::loadCalibrationFile(const std::string& path) {
    (void)path;
    spdlog::warn("{} is deprecated. This function call is replaced by Pipeline::setCalibrationData under pipeline. ", __func__);
}

void StereoDepth::setEmptyCalibration(void) {
    setRectification(false);
    spdlog::warn("{} is deprecated. This function call can be replaced by Stereo::setRectification(false). ", __func__);
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
    assetManager.set(assetKey, meshAsset);
    properties.mesh.meshLeftUri = std::string("asset:") + assetKey;

    meshAsset.data = dataRight;
    assetKey = "meshRight";
    assetManager.set(assetKey, meshAsset);
    properties.mesh.meshRightUri = std::string("asset:") + assetKey;

    properties.mesh.meshSize = meshAsset.data.size();
}

void StereoDepth::loadMeshFiles(const std::string& pathLeft, const std::string& pathRight) {
    std::ifstream streamLeft(pathLeft, std::ios::binary);
    if(!streamLeft.is_open()) {
        throw std::runtime_error("StereoDepth | Cannot open mesh at path: " + pathLeft);
    }
    std::vector<std::uint8_t> dataLeft = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamLeft), {});

    std::ifstream streamRight(pathRight, std::ios::binary);
    if(!streamRight.is_open()) {
        throw std::runtime_error("StereoDepth | Cannot open mesh at path: " + pathRight);
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
void StereoDepth::setOutputSize(int width, int height) {
    properties.outWidth = width;
    properties.outHeight = height;
}
void StereoDepth::setOutputKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
}
void StereoDepth::setMedianFilter(dai::MedianFilter median) {
    initialConfig.setMedianFilter(median);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setDepthAlign(Properties::DepthAlign align) {
    properties.depthAlign = align;
    // Unset 'depthAlignCamera', that would take precedence otherwise
    properties.depthAlignCamera = CameraBoardSocket::AUTO;
}
void StereoDepth::setDepthAlign(CameraBoardSocket camera) {
    properties.depthAlignCamera = camera;
}
void StereoDepth::setConfidenceThreshold(int confThr) {
    initialConfig.setConfidenceThreshold(confThr);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setRectification(bool enable) {
    properties.enableRectification = enable;
}
void StereoDepth::setLeftRightCheck(bool enable) {
    properties.enableLeftRightCheck = enable;
}
void StereoDepth::setSubpixel(bool enable) {
    properties.enableSubpixel = enable;
}
void StereoDepth::setExtendedDisparity(bool enable) {
    properties.enableExtendedDisparity = enable;
}
void StereoDepth::setRectifyEdgeFillColor(int color) {
    properties.rectifyEdgeFillColor = color;
}
void StereoDepth::setRectifyMirrorFrame(bool enable) {
    properties.rectifyMirrorFrame = enable;
}
void StereoDepth::setOutputRectified(bool enable) {
    (void)enable;
    spdlog::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}
void StereoDepth::setOutputDepth(bool enable) {
    (void)enable;
    spdlog::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}

float StereoDepth::getMaxDisparity() const {
    float maxDisp = 95.0;
    if(properties.enableExtendedDisparity) maxDisp *= 2;
    if(properties.enableSubpixel) maxDisp *= 32;
    return maxDisp;
}

}  // namespace node
}  // namespace dai

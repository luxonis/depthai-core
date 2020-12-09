#include "depthai/pipeline/node/StereoDepth.hpp"

// standard
#include <fstream>

namespace dai {
namespace node {

StereoDepth::StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    // 'properties' defaults already set
}

std::string StereoDepth::getName() const {
    return "StereoDepth";
}

std::vector<Node::Output> StereoDepth::getOutputs() {
    return {disparity, depth, syncedLeft, syncedRight, rectifiedLeft, rectifiedRight};
}

std::vector<Node::Input> StereoDepth::getInputs() {
    return {left, right};
}

nlohmann::json StereoDepth::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> StereoDepth::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void StereoDepth::loadCalibrationData(const std::vector<std::uint8_t>& data) {
    if(data.empty()) {
        // Will use EEPROM data
        properties.calibration.clear();
    } else {
        properties.calibration = data;
    }
}

void StereoDepth::loadCalibrationFile(const std::string& path) {
    std::vector<std::uint8_t> data;
    if(!path.empty()) {
        std::ifstream calib(path, std::ios::in | std::ios::binary);
        if(!calib.is_open()) {
            throw std::runtime_error("StereoDepth node | Unable to open calibration file: " + path);
        }
        data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(calib), {});
    }
    loadCalibrationData(data);
}

void StereoDepth::setEmptyCalibration(void) {
    // Special case: a single element
    const std::vector<std::uint8_t> empty = {0};
    properties.calibration = empty;
}

void StereoDepth::setInputResolution(int width, int height) {
    properties.width = width;
    properties.height = height;
}
void StereoDepth::setMedianFilter(StereoDepthProperties::MedianFilter median) {
    properties.median = median;
}
void StereoDepth::setConfidenceThreshold(int confThr) {
    properties.confidenceThreshold = confThr;
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
    properties.enableOutputRectified = enable;
}
void StereoDepth::setOutputDepth(bool enable) {
    properties.enableOutputDepth = enable;
}

}  // namespace node
}  // namespace dai

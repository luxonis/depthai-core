#include "depthai/pipeline/node/ImageManip.hpp"
namespace dai {
namespace node {

ImageManip::ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawImageManipConfig>()), config(rawConfig) {}

std::string ImageManip::getName() const {
    return "ImageManip";
}

std::vector<Node::Input> ImageManip::getInputs() {
    return {inputConfig, inputImage};
}

std::vector<Node::Output> ImageManip::getOutputs() {
    return {out};
}

nlohmann::json ImageManip::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> ImageManip::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

// Initial ImageManipConfig
void ImageManip::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    config.setCropRect(xmin, ymin, xmax, ymax);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setCropRotatedRect(RawImageManipConfig::RotatedRect rr, bool normalizedCoords) {
    config.setCropRotatedRect(rr, normalizedCoords);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setCenterCrop(float ratio, float whRatio) {
    config.setCenterCrop(ratio, whRatio);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setWarpTransformFourPoints(std::vector<RawImageManipConfig::Point2f> pt, bool normalizedCoords) {
    config.setWarpTransformFourPoints(pt, normalizedCoords);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setWarpTransformMatrix3x3(std::vector<float> mat) {
    config.setWarpTransformMatrix3x3(mat);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setRotationDegrees(float deg) {
    config.setRotationDegrees(deg);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setRotationRadians(float rad) {
    config.setRotationRadians(rad);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setResize(int w, int h) {
    config.setResize(w, h);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    config.setResizeThumbnail(w, h, bgRed, bgGreen, bgBlue);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setFrameType(dai::RawImgFrame::Type type) {
    config.setFrameType(type);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setHorizontalFlip(bool flip) {
    config.setHorizontalFlip(flip);
    properties.initialConfig = *rawConfig;
}

// Node properties configuration
void ImageManip::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

void ImageManip::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManip::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai

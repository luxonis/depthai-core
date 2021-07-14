#include "depthai/pipeline/node/ImageManip.hpp"
namespace dai {
namespace node {

ImageManip::ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawConfig(std::make_shared<RawImageManipConfig>()), initialConfig(rawConfig) {
    inputs = {&inputConfig, &inputImage};
    outputs = {&out};
}

std::string ImageManip::getName() const {
    return "ImageManip";
}

nlohmann::json ImageManip::getProperties() {
    nlohmann::json j;
    properties.initialConfig = *rawConfig;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> ImageManip::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

// Initial ImageManipConfig
void ImageManip::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    initialConfig.setCropRect(xmin, ymin, xmax, ymax);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setCenterCrop(float ratio, float whRatio) {
    initialConfig.setCenterCrop(ratio, whRatio);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setResize(int w, int h) {
    initialConfig.setResize(w, h);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    initialConfig.setResizeThumbnail(w, h, bgRed, bgGreen, bgBlue);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setFrameType(dai::RawImgFrame::Type type) {
    initialConfig.setFrameType(type);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setHorizontalFlip(bool flip) {
    initialConfig.setHorizontalFlip(flip);
    properties.initialConfig = *rawConfig;
}

void ImageManip::setKeepAspectRatio(bool keep) {
    initialConfig.setKeepAspectRatio(keep);
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

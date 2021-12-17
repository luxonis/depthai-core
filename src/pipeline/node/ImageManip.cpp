#include "depthai/pipeline/node/ImageManip.hpp"
namespace dai {
namespace node {

ImageManip::ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : ImageManip(par, nodeId, std::make_unique<ImageManip::Properties>()) {}
ImageManip::ImageManip(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, ImageManip, ImageManipProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawImageManipConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputImage});
    setOutputRefs({&out});
}

ImageManip::Properties& ImageManip::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
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
    inputConfig.setWaitForMessage(wait);
}

bool ImageManip::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void ImageManip::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManip::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai

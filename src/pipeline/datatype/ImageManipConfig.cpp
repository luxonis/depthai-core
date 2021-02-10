#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImageManipConfig::serialize() const {
    return raw;
}

ImageManipConfig::ImageManipConfig() : Buffer(std::make_shared<RawImageManipConfig>()), cfg(*dynamic_cast<RawImageManipConfig*>(raw.get())) {}
ImageManipConfig::ImageManipConfig(std::shared_ptr<RawImageManipConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawImageManipConfig*>(raw.get())) {}

// helpers
// Functions to set properties
void ImageManipConfig::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    // Enable crop stage
    cfg.enableCrop = true;

    // Disable center crop
    cfg.cropConfig.enableCenterCropRectangle = false;

    // Set crop rect
    cfg.cropConfig.cropRect.xmin = xmin;
    cfg.cropConfig.cropRect.ymin = ymin;
    cfg.cropConfig.cropRect.xmax = xmax;
    cfg.cropConfig.cropRect.ymax = ymax;
}

void ImageManipConfig::setCenterCrop(float ratio, float whRatio) {
    // Enable crop stage
    cfg.enableCrop = true;

    // Enable center center crop
    cfg.cropConfig.enableCenterCropRectangle = true;

    // Set crop center crop config
    cfg.cropConfig.cropRatio = ratio;
    cfg.cropConfig.widthHeightAspectRatio = whRatio;
}

void ImageManipConfig::setResize(int w, int h) {
    // Enable resize stage
    cfg.enableResize = true;

    // Disable lock aspect ratio
    cfg.resizeConfig.lockAspectRatioFill = false;

    // Set resize config
    cfg.resizeConfig.width = w;
    cfg.resizeConfig.height = h;
}

void ImageManipConfig::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    // Enable resize stage
    cfg.enableResize = true;

    // Set resize config
    cfg.resizeConfig.width = w;
    cfg.resizeConfig.height = h;

    // Set lock aspect ratio
    cfg.resizeConfig.lockAspectRatioFill = true;

    // Set background colors
    cfg.resizeConfig.bgRed = bgRed;
    cfg.resizeConfig.bgGreen = bgGreen;
    cfg.resizeConfig.bgBlue = bgBlue;
}

void ImageManipConfig::setFrameType(dai::RawImgFrame::Type type) {
    // Enable format stage
    cfg.enableFormat = true;

    // Set type format
    cfg.formatConfig.type = type;
}

void ImageManipConfig::setHorizontalFlip(bool flip) {
    // Enable format stage
    cfg.enableFormat = true;

    // Set pixel format
    cfg.formatConfig.flipHorizontal = flip;
}

void ImageManipConfig::setKeepAspectRatio(bool keep) {

    // Set whether to keep aspect ratio or not
    cfg.resizeConfig.keepAspectRatio = keep;
}


// Functions to retrieve properties
float ImageManipConfig::getCropXMin() const {
    return cfg.cropConfig.cropRect.xmin;
}

float ImageManipConfig::getCropYMin() const {
    return cfg.cropConfig.cropRect.ymin;
}

float ImageManipConfig::getCropXMax() const {
    return cfg.cropConfig.cropRect.xmax;
}

float ImageManipConfig::getCropYMax() const {
    return cfg.cropConfig.cropRect.ymax;
}

int ImageManipConfig::getResizeWidth() const {
    return cfg.resizeConfig.width;
}

int ImageManipConfig::getResizeHeight() const {
    return cfg.resizeConfig.height;
}

bool ImageManipConfig::isResizeThumbnail() const {
    return cfg.resizeConfig.lockAspectRatioFill;
}

}  // namespace dai

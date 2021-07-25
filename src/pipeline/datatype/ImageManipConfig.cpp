// First, as other headers may include <cmath>
#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

#include <cmath>

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

    // Set crop rect - limit to bounds beforehand
    cfg.cropConfig.cropRect.xmin = std::max(xmin, 0.0f);
    cfg.cropConfig.cropRect.ymin = std::max(ymin, 0.0f);
    cfg.cropConfig.cropRect.xmax = std::min(xmax, 1.0f);
    cfg.cropConfig.cropRect.ymax = std::min(ymax, 1.0f);
}

void ImageManipConfig::setCropRotatedRect(RotatedRect rr, bool normalizedCoords) {
    // Enable crop stage and extended flags
    cfg.enableCrop = true;
    cfg.cropConfig.enableRotatedRect = true;

    cfg.cropConfig.cropRotatedRect = rr;
    cfg.cropConfig.normalizedCoords = normalizedCoords;
}

void ImageManipConfig::setWarpTransformFourPoints(std::vector<Point2f> pt, bool normalizedCoords) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.enableWarp4pt = true;
    cfg.resizeConfig.warpFourPoints = pt;
    cfg.resizeConfig.normalizedCoords = normalizedCoords;
}

void ImageManipConfig::setWarpTransformMatrix3x3(std::vector<float> mat) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.enableWarpMatrix = true;
    cfg.resizeConfig.warpMatrix3x3 = mat;
}

void ImageManipConfig::setWarpBorderReplicatePixels() {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.warpBorderReplicate = true;
}

void ImageManipConfig::setWarpBorderFillColor(int red, int green, int blue) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.warpBorderReplicate = false;
    cfg.resizeConfig.bgRed = red;
    cfg.resizeConfig.bgGreen = green;
    cfg.resizeConfig.bgBlue = blue;
}

void ImageManipConfig::setCenterCrop(float ratio, float whRatio) {
    // Enable crop stage
    cfg.enableCrop = true;

    // Enable center center crop
    cfg.cropConfig.enableCenterCropRectangle = true;

    // Set crop center crop config
    cfg.cropConfig.cropRatio = ratio;
    // Limit to max 1.0f and disallow setting zero ratio
    if(ratio > 1.0f || ratio < 0.0f) {
        cfg.cropConfig.cropRatio = 1.0f;
    }

    cfg.cropConfig.widthHeightAspectRatio = whRatio;
}

void ImageManipConfig::setRotationDegrees(float deg) {
    cfg.enableResize = true;
    cfg.resizeConfig.rotationAngleDeg = deg;
    cfg.resizeConfig.enableRotation = true;
}

void ImageManipConfig::setRotationRadians(float rad) {
    static constexpr float rad2degFactor = static_cast<float>(180 / M_PI);
    setRotationDegrees(rad * rad2degFactor);
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

void ImageManipConfig::setReusePreviousImage(bool reuse) {
    cfg.reusePreviousImage = reuse;
}

void ImageManipConfig::setSkipCurrentImage(bool skip) {
    cfg.skipCurrentImage = skip;
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

// First, as other headers may include <cmath>
#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

#include <cmath>

namespace dai {

// helpers
std::tuple<float, float, float, float> getOuterRect(std::array<dai::Point2f, 4>& points) {
    float xmin = points[0].x;
    float ymin = points[0].y;
    float xmax = points[0].x;
    float ymax = points[0].y;
    for(int i = 2; i < 8; i += 2) {
        xmin = std::min(xmin, points[i].x);
        xmax = std::max(xmax, points[i].x);
        ymin = std::min(ymin, points[i].y);
        ymax = std::max(ymax, points[i].y);
    }
    return {xmin, ymin, xmax, ymax};
}

// New API
ImageManipConfig& ImageManipConfig::crop(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    base.crop(x, y, w, h);
    return *this;
}
ImageManipConfig& ImageManipConfig::resize(uint32_t w, uint32_t h) {
    base.resize(w, h);
    return *this;
}
ImageManipConfig& ImageManipConfig::scale(float scale) {
    base.resizeWidthKeepAspectRatio(scale, true);
    return *this;
}
ImageManipConfig& ImageManipConfig::rotateDeg(float angle) {
    base.rotateDegrees(angle);
    return *this;
}
ImageManipConfig& ImageManipConfig::setOutputSize(uint32_t w, uint32_t h, ImageManipBase::ResizeMode mode) {
    base.setOutputResize(w, h, mode);
    return *this;
}

// Functions to set properties
ImageManipConfig& ImageManipConfig::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    // Enable crop stage
    enableCrop = true;

    // Disable center crop
    cropConfig.enableCenterCropRectangle = false;

    // Set crop rect - limit to bounds beforehand
    cropConfig.cropRect.xmin = std::max(xmin, 0.0f);
    cropConfig.cropRect.ymin = std::max(ymin, 0.0f);
    cropConfig.cropRect.xmax = std::min(xmax, 1.0f);
    cropConfig.cropRect.ymax = std::min(ymax, 1.0f);

    base.translate(-xmin, -ymin, (xmax - xmin) <= 1.0f);
    base.setOutputSize(xmax - xmin, ymax - ymin);
    return *this;
}

ImageManipConfig& ImageManipConfig::setCropRect(std::tuple<float, float, float, float> coordinates) {
    setCropRect(std::get<0>(coordinates), std::get<1>(coordinates), std::get<2>(coordinates), std::get<3>(coordinates));
    return *this;
}

ImageManipConfig& ImageManipConfig::setCropRotatedRect(RotatedRect rr, bool normalizedCoords) {
    // Enable crop stage and extended flags
    enableCrop = true;
    cropConfig.enableRotatedRect = true;

    cropConfig.cropRotatedRect = rr;
    cropConfig.normalizedCoords = normalizedCoords;

    base.translate(-(rr.center.x - rr.size.width / 2), -(rr.center.y - rr.size.height / 2), normalizedCoords);
    base.setOutputSize(rr.size.width, rr.size.height);
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpTransformFourPoints(std::vector<Point2f> pt, bool normalizedCoords) {
    // Enable resize stage and extended flags
    enableResize = true;
    resizeConfig.keepAspectRatio = false;
    resizeConfig.enableWarp4pt = true;
    resizeConfig.warpFourPoints = pt;
    resizeConfig.normalizedCoords = normalizedCoords;

    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpTransformMatrix3x3(std::vector<float> mat) {
    // Enable resize stage and extended flags
    enableResize = true;
    resizeConfig.enableWarpMatrix = true;
    resizeConfig.warpMatrix3x3 = mat;

    base.transformPerspective({mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8]});

    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpBorderReplicatePixels() {
    // Enable resize stage and extended flags
    enableResize = true;
    resizeConfig.warpBorderReplicate = true;
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpBorderFillColor(int red, int green, int blue) {
    // Enable resize stage and extended flags
    enableResize = true;
    resizeConfig.warpBorderReplicate = false;
    resizeConfig.bgRed = red;
    resizeConfig.bgGreen = green;
    resizeConfig.bgBlue = blue;
    return *this;
}

ImageManipConfig& ImageManipConfig::setCenterCrop(float ratio, float whRatio) {
    // Enable crop stage
    enableCrop = true;

    // Enable center center crop
    cropConfig.enableCenterCropRectangle = true;

    // Set crop center crop config
    cropConfig.cropRatio = ratio;
    // Limit to max 1.0f and disallow setting zero ratio
    if(ratio > 1.0f || ratio < 0.0f) {
        cropConfig.cropRatio = 1.0f;
    }

    cropConfig.widthHeightAspectRatio = whRatio;

    base.setOutputSize(ratio * whRatio, ratio);
    base.setOutputCenter(true);

    return *this;
}

ImageManipConfig& ImageManipConfig::setRotationDegrees(float deg) {
    enableResize = true;
    resizeConfig.rotationAngleDeg = deg;
    resizeConfig.enableRotation = true;

    base.setOutputCenter(true);
    base.rotateDegrees(deg);

    return *this;
}

ImageManipConfig& ImageManipConfig::setRotationRadians(float rad) {
    static constexpr float rad2degFactor = static_cast<float>(180 / M_PI);
    setRotationDegrees(rad * rad2degFactor);

    base.setOutputCenter(true);
    base.rotateDegrees(rad);

    return *this;
}

ImageManipConfig& ImageManipConfig::setResize(int w, int h) {
    // Enable resize stage
    enableResize = true;

    // Disable lock aspect ratio
    resizeConfig.lockAspectRatioFill = false;

    // Set resize config
    resizeConfig.width = w;
    resizeConfig.height = h;

    base.resize(w, h);

    return *this;
}

ImageManipConfig& ImageManipConfig::setResize(std::tuple<int, int> size) {
    setResize(std::get<0>(size), std::get<1>(size));
    return *this;
}

ImageManipConfig& ImageManipConfig::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    // Enable resize stage
    enableResize = true;

    // Set resize config
    resizeConfig.width = w;
    resizeConfig.height = h;

    // Set lock aspect ratio
    resizeConfig.lockAspectRatioFill = true;

    // Set background colors
    resizeConfig.bgRed = bgRed;
    resizeConfig.bgGreen = bgGreen;
    resizeConfig.bgBlue = bgBlue;

    base.setBackgroundColor(bgRed, bgGreen, bgBlue);
    base.setOutputSize(w, h);
    base.setOutputCenter(true);
    base.resizeFit();

    return *this;
}

ImageManipConfig& ImageManipConfig::setResizeThumbnail(std::tuple<int, int> size, int bgRed, int bgGreen, int bgBlue) {
    setResizeThumbnail(std::get<0>(size), std::get<1>(size), bgRed, bgGreen, bgBlue);
    return *this;
}

ImageManipConfig& ImageManipConfig::setFrameType(ImgFrame::Type type) {
    // Enable format stage
    enableFormat = true;

    // Set type format
    formatConfig.type = type;
    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(Colormap colormap, float maxf) {
    int max = maxf;
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");

    // Enable format stage
    enableFormat = true;

    // Set type format
    formatConfig.colormap = colormap;
    formatConfig.colormapMin = 0;
    formatConfig.colormapMax = max;

    base.setColormap(colormap);

    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(Colormap colormap, int max) {
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");

    // Enable format stage
    enableFormat = true;

    // Set type format
    formatConfig.colormap = colormap;
    formatConfig.colormapMin = 0;
    formatConfig.colormapMax = max;

    base.colormap = colormap;

    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(Colormap colormap, int min, int max) {
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");
    if(min < 0 || min >= 256) throw std::invalid_argument("Colormap min argument must be between 0 and 255");

    // Enable format stage
    enableFormat = true;

    // Set type format
    formatConfig.colormap = colormap;
    formatConfig.colormapMin = min;
    formatConfig.colormapMax = max;

    base.colormap = colormap;

    return *this;
}

ImageManipConfig& ImageManipConfig::setHorizontalFlip(bool flip) {
    // Enable format stage
    enableFormat = true;

    // Set pixel format
    formatConfig.flipHorizontal = flip;

    base.flipHorizontal(true);

    return *this;
}

void ImageManipConfig::setVerticalFlip(bool flip) {
    // Enable format stage
    enableFormat = true;

    // Set pixel format
    formatConfig.flipVertical = flip;

    base.flipVertical(true);
}

ImageManipConfig& ImageManipConfig::setReusePreviousImage(bool reuse) {
    reusePreviousImage = reuse;
    return *this;
}

ImageManipConfig& ImageManipConfig::setSkipCurrentImage(bool skip) {
    skipCurrentImage = skip;
    return *this;
}

ImageManipConfig& ImageManipConfig::setKeepAspectRatio(bool keep) {
    // Set whether to keep aspect ratio or not
    resizeConfig.keepAspectRatio = keep;
    return *this;
}

ImageManipConfig& ImageManipConfig::setInterpolation(dai::Interpolation interpolation) {
    this->interpolation = interpolation;
    return *this;
}

// Functions to retrieve properties
float ImageManipConfig::getCropXMin() const {
    return cropConfig.cropRect.xmin;
}

float ImageManipConfig::getCropYMin() const {
    return cropConfig.cropRect.ymin;
}

float ImageManipConfig::getCropXMax() const {
    return cropConfig.cropRect.xmax;
}

float ImageManipConfig::getCropYMax() const {
    return cropConfig.cropRect.ymax;
}

int ImageManipConfig::getResizeWidth() const {
    return resizeConfig.width;
}

int ImageManipConfig::getResizeHeight() const {
    return resizeConfig.height;
}

ImageManipConfig::CropConfig ImageManipConfig::getCropConfig() const {
    return cropConfig;
}

ImageManipConfig::ResizeConfig ImageManipConfig::getResizeConfig() const {
    return resizeConfig;
}

ImageManipConfig::FormatConfig ImageManipConfig::getFormatConfig() const {
    return formatConfig;
}

bool ImageManipConfig::isResizeThumbnail() const {
    return resizeConfig.lockAspectRatioFill;
}

bool ImageManipConfig::getReusePreviousImage() const {
    return reusePreviousImage;
}

bool ImageManipConfig::getSkipCurrentImage() const {
    return skipCurrentImage;
}

Colormap ImageManipConfig::getColormap() const {
    return formatConfig.colormap;
}

dai::Interpolation ImageManipConfig::getInterpolation() const {
    return interpolation;
}

}  // namespace dai

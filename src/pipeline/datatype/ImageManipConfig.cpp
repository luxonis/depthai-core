// First, as other headers may include <cmath>
#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

#include <cmath>

namespace dai {

OpBase::~OpBase() = default;
Translate::~Translate() = default;
Rotate::~Rotate() = default;
Resize::~Resize() = default;
Flip::~Flip() = default;
Affine::~Affine() = default;
Perspective::~Perspective() = default;
FourPoints::~FourPoints() = default;
Crop::~Crop() = default;

ImageManipConfig::~ImageManipConfig() = default;

void ImageManipConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ImageManipConfig;
}

// New API
ImageManipConfig& ImageManipConfig::clearOps() {
    base.clear();
    return *this;
}
ImageManipConfig& ImageManipConfig::addCrop(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    base.crop(x, y, w, h);
    return *this;
}
ImageManipConfig& ImageManipConfig::addCrop(dai::Rect rect, bool normalizedCoords) {
    base.crop(rect.x, rect.y, rect.width, rect.height, normalizedCoords);
    return *this;
}
ImageManipConfig& ImageManipConfig::addCropRotatedRect(dai::RotatedRect rotatedRect, bool normalizedCoords) {
    base.rotateDegrees(-rotatedRect.angle);
    base.crop(rotatedRect.center.x - rotatedRect.size.width / 2,
              rotatedRect.center.y - rotatedRect.size.height / 2,
              rotatedRect.size.width,
              rotatedRect.size.height,
              normalizedCoords);
    return *this;
}
ImageManipConfig& ImageManipConfig::addScale(float scaleX, float scaleY) {
    base.resize(scaleX, scaleY, true);
    return *this;
}
ImageManipConfig& ImageManipConfig::addRotateDeg(float angle) {
    base.rotateDegrees(angle);
    return *this;
}
ImageManipConfig& ImageManipConfig::addRotateDeg(float angle, Point2f centerOffset) {
    base.rotateDegrees(angle, true, centerOffset.x, centerOffset.y, true);
    return *this;
}
ImageManipConfig& ImageManipConfig::addFlipHorizontal() {
    base.flipHorizontal(true);
    return *this;
}
ImageManipConfig& ImageManipConfig::addFlipVertical() {
    base.flipVertical(true);
    return *this;
}
ImageManipConfig& ImageManipConfig::addTransformAffine(std::array<float, 4> matrix) {
    base.transformAffine(matrix);
    return *this;
}
ImageManipConfig& ImageManipConfig::addTransformPerspective(std::array<float, 9> matrix) {
    base.transformPerspective(matrix);
    return *this;
}
ImageManipConfig& ImageManipConfig::addTransformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords) {
    base.transformFourPoints(src, dst, normalizedCoords);
    return *this;
}
ImageManipConfig& ImageManipConfig::setOutputSize(uint32_t w, uint32_t h, ResizeMode mode) {
    base.setOutputResize(w, h, mode);
    return *this;
}
ImageManipConfig& ImageManipConfig::setOutputCenter(bool c) {
    base.setOutputResize(0, 0, ResizeMode::NONE);
    base.setOutputCenter(c);
    return *this;
}
ImageManipConfig& ImageManipConfig::setColormap(Colormap colormap) {
    base.setColormap(colormap);
    return *this;
}
ImageManipConfig& ImageManipConfig::setBackgroundColor(uint32_t red, uint32_t green, uint32_t blue) {
    base.setBackgroundColor(red, green, blue);
    return *this;
}
ImageManipConfig& ImageManipConfig::setBackgroundColor(uint32_t val) {
    base.setBackgroundColor(val);
    return *this;
}
ImageManipConfig& ImageManipConfig::setFrameType(ImgFrame::Type frameType) {
    outputFrameType = frameType;
    return *this;
}

ImageManipConfig& ImageManipConfig::setUndistort(bool undistort) {
    base.setUndistort(undistort);
    return *this;
}

bool ImageManipConfig::getUndistort() const {
    return base.getUndistort();
}

ImageManipConfig& ImageManipConfig::setReusePreviousImage(bool reuse) {
    reusePreviousImage = reuse;
    return *this;
}

ImageManipConfig& ImageManipConfig::setSkipCurrentImage(bool skip) {
    skipCurrentImage = skip;
    return *this;
}

bool ImageManipConfig::getReusePreviousImage() const {
    return reusePreviousImage;
}

bool ImageManipConfig::getSkipCurrentImage() const {
    return skipCurrentImage;
}

}  // namespace dai

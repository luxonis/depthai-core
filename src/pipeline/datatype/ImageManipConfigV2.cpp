// First, as other headers may include <cmath>
#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"

#include <cmath>

namespace dai {

// New API
ImageManipConfigV2& ImageManipConfigV2::crop(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    base.crop(x, y, w, h);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::cropRotatedRect(dai::RotatedRect rotatedRect, bool normalizedCoords) {
    base.rotateDegrees(-rotatedRect.angle);
    base.crop(rotatedRect.center.x - rotatedRect.size.width / 2,
              rotatedRect.center.y - rotatedRect.size.height / 2,
              rotatedRect.size.width,
              rotatedRect.size.height,
              normalizedCoords);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::resize(uint32_t w, uint32_t h) {
    base.resize(w, h);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::scale(float scaleX, float scaleY) {
    base.resize(scaleX, scaleY, true);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::rotateDeg(float angle) {
    base.rotateDegrees(angle);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::rotateDeg(float angle, Point2f center) {
    base.translate(-center.x, -center.y, true);
    base.rotateDegrees(angle);
    base.translate(center.x, center.y, true);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::flipHorizontal() {
    base.flipHorizontal(true);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::flipVertical() {
    base.flipVertical(true);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::transformAffine(std::array<float, 4> matrix) {
    base.transformAffine(matrix);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::transformPerspective(std::array<float, 9> matrix) {
    base.transformPerspective(matrix);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::transformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords) {
    base.transformFourPoints(src, dst, normalizedCoords);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::setOutputSize(uint32_t w, uint32_t h, ResizeMode mode) {
    base.setOutputResize(w, h, mode);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::setColormap(Colormap colormap) {
    base.setColormap(colormap);
    return *this;
}
ImageManipConfigV2& ImageManipConfigV2::setFrameType(ImgFrame::Type frameType) {
    outputFrameType = frameType;
    return *this;
}

ImageManipConfigV2& ImageManipConfigV2::setReusePreviousImage(bool reuse) {
    reusePreviousImage = reuse;
    return *this;
}

ImageManipConfigV2& ImageManipConfigV2::setSkipCurrentImage(bool skip) {
    skipCurrentImage = skip;
    return *this;
}

bool ImageManipConfigV2::getReusePreviousImage() const {
    return reusePreviousImage;
}

bool ImageManipConfigV2::getSkipCurrentImage() const {
    return skipCurrentImage;
}

}  // namespace dai

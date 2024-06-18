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
    base.translate(-center.x, -center.y);
    base.rotateDegrees(angle);
    base.translate(center.x, center.y);
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

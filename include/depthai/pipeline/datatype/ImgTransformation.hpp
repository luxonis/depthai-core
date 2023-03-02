#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai-shared/common/ImgTransformation.hpp"

namespace dai {
class ImgTransformation{
   public:
    ImgTransformation() = delete;
    ImgTransformation(std::vector<RawImgTransformation> rawTransformations);
    virtual ~ImgTransformation() = default;

    void setSize(int width, int height);
    void setPadding(ImgTransformation sourceFrame, dai::Rect iRoiRect, int topPadding, int bottomPadding, int leftPadding, int rightPadding);
    void setCrop(ImgTransformation sourceFrame, dai::Rect crop);
    void setRotation(ImgTransformation sourceFrame, float rotationAngle);
    void setScale(ImgTransformation sourceFrame, float scaleFactorX, float scaleFactorY);
};
}  // namespace dai

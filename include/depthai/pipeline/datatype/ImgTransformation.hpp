#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai-shared/datatype/RawImgTransformation.hpp"

namespace dai {
class ImgTransformation{
    // std::shared_ptr<RawBuffer> serialize() const override;
    RawImgTransformation& transformation;
   public:
    ImgTransformation() = delete;
    explicit ImgTransformation(RawImgTransformation& transformation);
    virtual ~ImgTransformation() = default;

    dai::RawImgTransformation& get();
    void setPadding(ImgTransformation sourceFrame, dai::Rect iRoiRect, int topPadding, int bottomPadding, int leftPadding, int rightPadding);
    void setCrop(ImgTransformation sourceFrame, dai::Rect crop);
    void setRotation(ImgTransformation sourceFrame, float rotationAngle);
    void setScale(ImgTransformation sourceFrame, float scaleFactorX, float scaleFactorY);
};
}  // namespace dai
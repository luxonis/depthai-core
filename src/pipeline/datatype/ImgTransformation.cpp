#include "depthai/pipeline/datatype/ImgTransformation.hpp"

namespace dai {
// ImgTransformation::ImgTransformation(RawImgTransformation& transformation) : transformation(transformation){}
ImgTransformation::ImgTransformation(std::vector<RawImgTransformation> rawTransformations) {
    ;
}
//
void ImgTransformation::setSize(int width, int height){
    // transformation.crop.x = 0;
    // transformation.crop.y = 0;
    // transformation.crop.width = width;
    // transformation.crop.height = height;
}

void ImgTransformation::setPadding(ImgTransformation sourceFrame, dai::Rect iRoiRect, int topPadding, int bottomPadding, int leftPadding, int rightPadding) {
    // int sourceTopPadding = sourceFrame.get().topPadding;
    // int sourceLeftPadding = sourceFrame.get().leftPadding;
    // int sourceBottomPadding = sourceFrame.get().bottomPadding;
    // int sourceRightPadding = sourceFrame.get().rightPadding;

    // int topPaddingIntersection = 0;
    // int leftPaddingIntersection = 0;
    // int bottomPaddingIntersection = 0;
    // int rightPaddingIntersection = 0;

    // if((int)iRoiRect.y < sourceTopPadding) topPaddingIntersection = sourceTopPadding - iRoiRect.y;
    // if((int)iRoiRect.x < sourceLeftPadding) leftPaddingIntersection = sourceLeftPadding - iRoiRect.x;
    // if((int)iRoiRect.y + iRoiRect.height < sourceBottomPadding) bottomPaddingIntersection = sourceBottomPadding - iRoiRect.y + iRoiRect.height;
    // if((int)iRoiRect.x + iRoiRect.width < sourceRightPadding) rightPaddingIntersection = sourceRightPadding - iRoiRect.x + iRoiRect.width;

    // transformation.leftPadding = leftPadding + leftPaddingIntersection;
    // transformation.topPadding = topPadding + topPaddingIntersection;
    // transformation.rightPadding = rightPadding + rightPaddingIntersection;
    // transformation.bottomPadding = bottomPadding + bottomPaddingIntersection;
}

void ImgTransformation::setCrop(ImgTransformation sourceFrame, dai::Rect crop) {
    // transformation.crop.x = sourceFrame.get().crop.x + crop.x;
    // transformation.crop.y = sourceFrame.get().crop.y + crop.y;
    // transformation.crop.width = crop.width;
    // transformation.crop.height = crop.height;
}

void ImgTransformation::setRotation(ImgTransformation sourceFrame, float rotationAngle) {
    // transformation.rotationAngle = sourceFrame.get().rotationAngle + rotationAngle;
}

void ImgTransformation::setScale(ImgTransformation sourceFrame, float scaleFactorX, float scaleFactorY) {
    // transformation.scaleFactorX = sourceFrame.get().scaleFactorX * scaleFactorX;
    // transformation.scaleFactorY = sourceFrame.get().scaleFactorY * scaleFactorY;
}
}  // namespace dai

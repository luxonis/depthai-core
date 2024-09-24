#pragma once

#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {
struct ImgTransformation {
   private:
    std::array<std::array<float, 3>, 3> transformationMatrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> transformationMatrixInv = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};  // Precomputed inverse matrix
    size_t srcWidth = 0;
    size_t srcHeight = 0;
    size_t width = 0;
    size_t height = 0;

    std::vector<dai::RotatedRect> srcCrops = {};

    // To optimize mask calculations
    std::vector<uint8_t> srcMask;
    bool srcMaskValid = false;
    size_t srcMinX = 0;
    size_t srcMinY = 0;
    size_t srcMaxX = 0;
    size_t srcMaxY = 0;
    bool srcBorderValid = false;
    bool srcBorderComplex = false;
    std::vector<uint8_t> dstMask;
    bool dstMaskValid = false;
    size_t dstMinX = 0;
    size_t dstMinY = 0;
    size_t dstMaxX = 0;
    size_t dstMaxY = 0;
    bool dstBorderValid = false;
    bool dstBorderComplex = false;

    void calcSrcBorder();
    void calcDstBorder();

   public:
    ImgTransformation() = default;
    ImgTransformation(size_t width, size_t height) : srcWidth(width), srcHeight(height), width(width), height(height) {}

    dai::Point2f transformPoint(dai::Point2f point) const;
    dai::RotatedRect transformRect(dai::RotatedRect rect) const;
    dai::Point2f invTransformPoint(dai::Point2f point) const;
    dai::RotatedRect invTransformRect(dai::RotatedRect rect) const;

    std::pair<size_t, size_t> getSize() const;
    std::pair<size_t, size_t> getSourceSize() const;
    std::array<std::array<float, 3>, 3> getMatrix() const;

    bool getSrcMaskPt(size_t x, size_t y);
    bool getDstMaskPt(size_t x, size_t y);
    const std::vector<uint8_t>& getSrcMask(size_t srcWidth, size_t srcHeight);
    const std::vector<uint8_t>& getDstMask();

    ImgTransformation& addTransformation(std::array<std::array<float, 3>, 3> matrix);
    ImgTransformation& addCrop(int x, int y, int width, int height);
    ImgTransformation& addFlipVertical();
    ImgTransformation& addFlipHorizontal();
    ImgTransformation& addRotation(float angle, dai::Point2f rotationPoint);
    ImgTransformation& addScale(float scaleX, float scaleY);
    
    bool isValid() const;

    DEPTHAI_SERIALIZE(ImgTransformation, transformationMatrix, transformationMatrixInv, srcWidth, srcHeight, width, height, srcCrops);
};

// class ImgTransformations {
//    public:
//     std::vector<ImgTransformation> transformations = {};
//
//     bool invalidFlag = false;
//
//     unsigned int getLastWidth() const;
//
//     unsigned int getLastHeight() const;
//
//     void addPadding(int topPadding, int bottomPadding, int leftPadding, int rightPadding);
//
//     void addCrop(int topLeftCropX = 0, int topLeftCropY = 0, int bottomRightCropX = 0, int bottomRightCropY = 0);
//
//     void addFlipVertical();
//
//     void addFlipHorizontal();
//
//     void addInitTransformation(int width, int height);
//
//     void addRotation(float angle, dai::Point2f rotationPoint, int newWidth = 0, int newHeight = 0);
//
//     void addScale(float scaleX, float scaleY);
//
//     bool validateTransformationSizes() const;
//
//     // API that is meant for performance reasons - so matrices can be precomputed.
//     void addTransformation(std::vector<std::vector<float>> matrix,
//                            std::vector<std::vector<float>> invMatrix,
//                            ImgTransformation::Transformation transformation,
//                            int newWidth,
//                            int newHeight);
//
//    private:
//     ImgTransformation getNewTransformation() const;
//
//     static std::vector<std::vector<float>> getFlipHorizontalMatrix(int width);
//
//     static std::vector<std::vector<float>> getFlipVerticalMatrix(int height);
//
//     static std::vector<std::vector<float>> getRotationMatrix(int px, int py, float theta);
//
//     static std::vector<std::vector<float>> getScaleMatrix(float scaleX, float scaleY);
// };
//
// DEPTHAI_SERIALIZE_EXT(ImgTransformations, transformations, invalidFlag);

}  // namespace dai

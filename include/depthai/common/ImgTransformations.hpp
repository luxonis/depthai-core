#pragma once

#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix);

/**
    * ImgTransformation struct. Holds information of how a ImgFrame or related message was transformed from their source. Useful for remapping from one ImgFrame to another.
*/
struct ImgTransformation {
   private:
    std::array<std::array<float, 3>, 3> transformationMatrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> transformationMatrixInv = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};  // Precomputed inverse matrix
    std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> sourceIntrinsicMatrixInv = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
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
    ImgTransformation(size_t srcWidth, size_t srcHeight, size_t width, size_t height)
        : srcWidth(srcWidth), srcHeight(srcHeight), width(width), height(height) {}
    ImgTransformation(size_t width, size_t height, std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix)
        : sourceIntrinsicMatrix(sourceIntrinsicMatrix), srcWidth(width), srcHeight(height), width(width), height(height) {
        sourceIntrinsicMatrixInv = getMatrixInverse(sourceIntrinsicMatrix);
    }
    ImgTransformation(size_t srcWidth, size_t srcHeight, size_t width, size_t height, std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix)
        : sourceIntrinsicMatrix(sourceIntrinsicMatrix), srcWidth(srcWidth), srcHeight(srcHeight), width(width), height(height) {
        sourceIntrinsicMatrixInv = getMatrixInverse(sourceIntrinsicMatrix);
    }

    dai::Point2f transformPoint(dai::Point2f point) const;
    dai::RotatedRect transformRect(dai::RotatedRect rect) const;
    dai::Point2f invTransformPoint(dai::Point2f point) const;
    dai::RotatedRect invTransformRect(dai::RotatedRect rect) const;

    std::pair<size_t, size_t> getSize() const;
    std::pair<size_t, size_t> getSourceSize() const;
    std::array<std::array<float, 3>, 3> getMatrix() const;
    std::array<std::array<float, 3>, 3> getMatrixInv() const;
    std::array<std::array<float, 3>, 3> getSourceIntrinsicMatrix() const;
    std::array<std::array<float, 3>, 3> getSourceIntrinsicMatrixInv() const;
    std::vector<dai::RotatedRect> getSrcCrops() const;

    bool getSrcMaskPt(size_t x, size_t y);
    bool getDstMaskPt(size_t x, size_t y);
    const std::vector<uint8_t>& getSrcMask(size_t srcWidth, size_t srcHeight);
    const std::vector<uint8_t>& getDstMask();

    ImgTransformation& addTransformation(std::array<std::array<float, 3>, 3> matrix);
    ImgTransformation& addCrop(int x, int y, int width, int height);
    ImgTransformation& addPadding(int x, int y, int width, int height);
    ImgTransformation& addFlipVertical();
    ImgTransformation& addFlipHorizontal();
    ImgTransformation& addRotation(float angle, dai::Point2f rotationPoint);
    ImgTransformation& addScale(float scaleX, float scaleY);
    ImgTransformation& addSrcCrops(const std::vector<dai::RotatedRect>& crops);

    dai::Point2f remapPointTo(const ImgTransformation& to, dai::Point2f point) const;
    dai::Point2f remapPointTo(const std::array<std::array<float, 3>, 3>& to, dai::Point2f point) const;
    dai::Point2f remapPointFrom(const ImgTransformation& from, dai::Point2f point) const;
    dai::Point2f remapPointFrom(const std::array<std::array<float, 3>, 3>& from, dai::Point2f point) const;
    dai::RotatedRect remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const;
    dai::RotatedRect remapRectTo(const std::array<std::array<float, 3>, 3>& to, dai::RotatedRect rect) const;
    dai::RotatedRect remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const;
    dai::RotatedRect remapRectFrom(const std::array<std::array<float, 3>, 3>& from, dai::RotatedRect rect) const;

    bool isValid() const;

    std::string str() const;

    DEPTHAI_SERIALIZE(ImgTransformation, transformationMatrix, transformationMatrixInv, sourceIntrinsicMatrix, sourceIntrinsicMatrixInv, srcWidth, srcHeight, width, height, srcCrops);
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

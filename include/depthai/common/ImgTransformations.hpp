#pragma once

#include "depthai/common/CameraModel.hpp"
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
    CameraModel distortionModel;
    std::vector<float> distortionCoefficients;
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
    std::vector<uint8_t> dstMask;
    bool dstMaskValid = false;
    size_t dstMinX = 0;
    size_t dstMinY = 0;
    size_t dstMaxX = 0;
    size_t dstMaxY = 0;
    bool dstBorderValid = false;

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
    ImgTransformation(size_t width, size_t height, std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix, CameraModel distortionModel, std::vector<float> distortionCoefficients)
        : sourceIntrinsicMatrix(sourceIntrinsicMatrix), distortionModel(distortionModel), distortionCoefficients(distortionCoefficients), srcWidth(width), srcHeight(height), width(width), height(height) {
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

    DEPTHAI_SERIALIZE(ImgTransformation, transformationMatrix, transformationMatrixInv, sourceIntrinsicMatrix, sourceIntrinsicMatrixInv, distortionModel, distortionCoefficients, srcWidth, srcHeight, width, height, srcCrops);
};

}  // namespace dai

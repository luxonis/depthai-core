#pragma once

#include "depthai/common/CameraModel.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix);

/**
 * ImgTransformation struct. Holds information of how a ImgFrame or related message was transformed from their source. Useful for remapping from one ImgFrame to
 * another.
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
    ImgTransformation(size_t width,
                      size_t height,
                      std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix,
                      CameraModel distortionModel,
                      std::vector<float> distortionCoefficients)
        : sourceIntrinsicMatrix(sourceIntrinsicMatrix),
          distortionModel(distortionModel),
          distortionCoefficients(distortionCoefficients),
          srcWidth(width),
          srcHeight(height),
          width(width),
          height(height) {
        sourceIntrinsicMatrixInv = getMatrixInverse(sourceIntrinsicMatrix);
    }

    /**
     * Transform a point from the source frame to the current frame.
     * @param point Point to transform
     * @return Transformed point
     */
    dai::Point2f transformPoint(dai::Point2f point) const;
    /**
     * Transform a rotated rect from the source frame to the current frame.
     * @param rect Rectangle to transform
     * @return Transformed rectangle
     */
    dai::RotatedRect transformRect(dai::RotatedRect rect) const;
    /**
     * Transform a point from the current frame to the source frame.
     * @param point Point to transform
     * @return Transformed point
     */
    dai::Point2f invTransformPoint(dai::Point2f point) const;
    /**
     * Transform a rotated rect from the current frame to the source frame.
     * @param rect Rectangle to transform
     * @return Transformed rectangle
     */
    dai::RotatedRect invTransformRect(dai::RotatedRect rect) const;

    /**
     * Retrieve the size of the frame. Should be equal to the size of the corresponding ImgFrame message.
     * @return Size of the frame
     */
    std::pair<size_t, size_t> getSize() const;
    /**
     * Retrieve the size of the source frame from which this frame was derived.
     * @return Size of the frame
     */
    std::pair<size_t, size_t> getSourceSize() const;
    /**
     * Retrieve the transformation matrix from the source frame to the current frame.
     * @return Transformation matrix
     */
    std::array<std::array<float, 3>, 3> getMatrix() const;
    /**
     * Retrieve the inverse transformation matrix from the current frame to the source frame.
     * @return Inverse transformation matrix
     */
    std::array<std::array<float, 3>, 3> getMatrixInv() const;
    /**
     * Retrieve the intrinsic matrix of the source sensor.
     * @return Intrinsic matrix
     */
    std::array<std::array<float, 3>, 3> getSourceIntrinsicMatrix() const;
    /**
     * Retrieve the inverse intrinsic matrix of the source sensor.
     * @return Inverse intrinsic matrix
     */
    std::array<std::array<float, 3>, 3> getSourceIntrinsicMatrixInv() const;
    std::vector<dai::RotatedRect> getSrcCrops() const;

    bool getSrcMaskPt(size_t x, size_t y);
    bool getDstMaskPt(size_t x, size_t y);

    /**
     * Add a new transformation.
     * @param matrix Transformation matrix
     */
    ImgTransformation& addTransformation(std::array<std::array<float, 3>, 3> matrix);
    /**
     * Add a crop transformation.
     * @param x X coordinate of the top-left corner of the crop
     * @param y Y coordinate of the top-left corner of the crop
     * @param width Width of the crop
     * @param height Height of the crop
     */
    ImgTransformation& addCrop(int x, int y, int width, int height);
    /**
     * Add a pad transformation. Works like crop, but in reverse.
     * @param x Padding on the left. The x coordinate of the top-left corner in the new image.
     * @param y Padding on the top. The y coordinate of the top-left corner in the new image.
     * @param width New image width
     * @param height New image height
     */
    ImgTransformation& addPadding(int x, int y, int width, int height);
    /**
     * Add a vertical flip transformation.
     */
    ImgTransformation& addFlipVertical();
    /**
     * Add a horizontal flip transformation.
     */
    ImgTransformation& addFlipHorizontal();
    /**
     * Add a rotation transformation.
     * @param angle Angle in degrees
     * @param rotationPoint Point around which to rotate
     */
    ImgTransformation& addRotation(float angle, dai::Point2f rotationPoint);
    /**
     * Add a scale transformation.
     * @param scaleX Scale factor in the horizontal direction
     * @param scaleY Scale factor in the vertical direction
     */
    ImgTransformation& addScale(float scaleX, float scaleY);
    ImgTransformation& addSrcCrops(const std::vector<dai::RotatedRect>& crops);

    /**
     * Remap a point from this transformation to another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to remap the point.
     * @param to Transformation to remap to
     * @param point Point to remap
     */
    dai::Point2f remapPointTo(const ImgTransformation& to, dai::Point2f point) const;
    /**
     * Remap a point to this transformation from another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to remap the point.
     * @param from Transformation to remap from
     * @param point Point to remap
     */
    dai::Point2f remapPointFrom(const ImgTransformation& from, dai::Point2f point) const;
    /**
     * Remap a rotated rect from this transformation to another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to remap the rect.
     * @param to Transformation to remap to
     * @param rect RotatedRect to remap
     */
    dai::RotatedRect remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const;
    /**
     * Remap a rotated rect to this transformation from another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to remap the rect.
     * @param from Transformation to remap from
     * @param point RotatedRect to remap
     */
    dai::RotatedRect remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const;

    /**
     * Check if the transformations are valid. The transformations are valid if the source frame size and the current frame size are set.
     */
    bool isValid() const;

    DEPTHAI_SERIALIZE(ImgTransformation,
                      transformationMatrix,
                      transformationMatrixInv,
                      sourceIntrinsicMatrix,
                      sourceIntrinsicMatrixInv,
                      distortionModel,
                      distortionCoefficients,
                      srcWidth,
                      srcHeight,
                      width,
                      height,
                      srcCrops);
};

}  // namespace dai

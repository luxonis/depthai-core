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
    CameraModel distortionModel = CameraModel::Perspective;
    std::vector<float> distortionCoefficients;
    size_t srcWidth = 0;
    size_t srcHeight = 0;
    size_t width = 0;
    size_t height = 0;

    std::vector<dai::RotatedRect> srcCrops = {};

    dai::RotatedRect srcCrop;
    dai::RotatedRect dstCrop;
    bool cropsValid = false;

    void calcCrops();

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
    /**
     * Retrieve the distortion model of the source sensor
     * @return Distortion model
     */
    CameraModel getDistortionModel() const;
    /**
     * Retrieve the distortion coefficients of the source sensor
     * @return vector of distortion coefficients
     */
    std::vector<float> getDistortionCoefficients() const;
    /**
     * Retrieve the total intrinsic matrix calculated from intrinsic * transform.
     * @return total intrinsic matrix
     */
    std::array<std::array<float, 3>, 3> getIntrinsicMatrix() const;
    /**
     * Retrieve the inverse of the total intrinsic matrix calculated from intrinsic * transform.
     * @return inverse total intrinsic matrix
     */
    std::array<std::array<float, 3>, 3> getIntrinsicMatrixInv() const;
    /**
     * Retrieve the diagonal field of view of the image.
     * @param source If true, the source field of view will be returned. Otherwise, the current field of view will be returned.
     * @return Diagonal field of view in degrees
     */
    float getDFov(bool source = false) const;
    /**
     * Retrieve the horizontal field of view of the image.
     * @param source If true, the source field of view will be returned. Otherwise, the current field of view will be returned.
     * @return Horizontal field of view in degrees
     */
    float getHFov(bool source = false) const;
    /**
     * Retrieve the vertical field of view of the image.
     * @param source If true, the source field of view will be returned. Otherwise, the current field of view will be returned.
     * @return Vertical field of view in degrees
     */
    float getVFov(bool source = false) const;

    /**
     * Retrieve currently set source crop rectangles.
     * @return Vector of source crops
     */
    std::vector<dai::RotatedRect> getSrcCrops() const;

    /**
     * Returns true if the point is inside the transformed region of interest (determined by crops used).
     */
    bool getSrcMaskPt(size_t x, size_t y);
    /**
     * Returns true if the point is inside the image region (not in the background region).
     */
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
     * @param top Padding on the top
     * @param bottom Padding on the bottom
     * @param left Padding on the left
     * @param right Padding on the right
     */
    ImgTransformation& addPadding(int top, int bottom, int left, int right);
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
    /**
     * Append source crop rectangles.
     * @param crops Vector of source crops to add
     */
    ImgTransformation& addSrcCrops(const std::vector<dai::RotatedRect>& crops);
    /**
     * Set output image size.
     * @param width Output width
     * @param height Output height
     */
    ImgTransformation& setSize(size_t width, size_t height);
    /**
     * Set input image size.
     * @param width Input width
     * @param height Input height
     */
    ImgTransformation& setSourceSize(size_t width, size_t height);
    /**
     * Set source intrinsic matrix.
     * @param intrinsicMatrix 3x3 intrinsic matrix
     */
    ImgTransformation& setIntrinsicMatrix(std::array<std::array<float, 3>, 3> intrinsicMatrix);
    /**
     * Set the source distortion model.
     * @param model Distortion model
     */
    ImgTransformation& setDistortionModel(CameraModel model);
    /**
     * Set the source distortion coefficients.
     * @param coefficients Distortion coefficients
     */
    ImgTransformation& setDistortionCoefficients(std::vector<float> coefficients);

    /**
     * Remap a point from this transformation to another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to
     * remap the point.
     * @param to Transformation to remap to
     * @param point Point to remap
     */
    dai::Point2f remapPointTo(const ImgTransformation& to, dai::Point2f point) const;
    /**
     * Remap a point to this transformation from another. If the intrinsics are different (e.g. different camera), the function will also use the intrinsics to
     * remap the point.
     * @param from Transformation to remap from
     * @param point Point to remap
     */
    dai::Point2f remapPointFrom(const ImgTransformation& from, dai::Point2f point) const;
    /**
     * Remap a rotated rect from this transformation to another. If the intrinsics are different (e.g. different camera), the function will also use the
     * intrinsics to remap the rect.
     * @param to Transformation to remap to
     * @param rect RotatedRect to remap
     */
    dai::RotatedRect remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const;
    /**
     * Remap a rotated rect to this transformation from another. If the intrinsics are different (e.g. different camera), the function will also use the
     * intrinsics to remap the rect.
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

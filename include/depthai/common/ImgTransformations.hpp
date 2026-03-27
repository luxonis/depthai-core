#pragma once

#include <array>

#include "depthai/common/CameraModel.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {

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
    Extrinsics extrinsics = {};

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
        sourceIntrinsicMatrixInv = matrix::getMatrixInverse(sourceIntrinsicMatrix);
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
        sourceIntrinsicMatrixInv = matrix::getMatrixInverse(sourceIntrinsicMatrix);
    }

    ImgTransformation(size_t width,
                      size_t height,
                      std::array<std::array<float, 3>, 3> sourceIntrinsicMatrix,
                      CameraModel distortionModel,
                      std::vector<float> distortionCoefficients,
                      Extrinsics extrinsics)
        : sourceIntrinsicMatrix(sourceIntrinsicMatrix),
          distortionModel(distortionModel),
          distortionCoefficients(std::move(distortionCoefficients)),
          extrinsics(std::move(extrinsics)),
          srcWidth(width),
          srcHeight(height),
          width(width),
          height(height) {
        sourceIntrinsicMatrixInv = matrix::getMatrixInverse(sourceIntrinsicMatrix);
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
     * Retrieve the extrinsics to the source sensor.
     * @return Extrinsics
     */
    Extrinsics getExtrinsics() const;

    /**
     * Two transformations are equal if the transformation matrices, intrinsic matrices, distortion models,
     * distortion coefficients, extrinsics, and sizes are all equal.
     * @param other Transformation to compare with
     * @return True if the transformations are equal, false otherwise
     */
    bool isEqualTransformation(const ImgTransformation& other) const;

    std::array<std::array<float, 3>, 3> getTransformationMatrix() const {
        return transformationMatrix;
    }

    std::array<std::array<float, 3>, 3> getTransformationMatrixInv() const {
        return transformationMatrixInv;
    }

    /**
     * Retrieve the total intrinsic matrix calculated from transform * intrinsic.
     * @return total intrinsic matrix
     */
    std::array<std::array<float, 3>, 3> getIntrinsicMatrix() const;
    /**
     * Retrieve the inverse of the total intrinsic matrix.
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
    ImgTransformation& addSrcCrops(const std::vector<dai::RotatedRect>& crops);
    ImgTransformation& setSize(size_t width, size_t height);
    ImgTransformation& setSourceSize(size_t width, size_t height);
    ImgTransformation& setExtrinsics(const Extrinsics& extrinsics);
    ImgTransformation& setIntrinsicMatrix(std::array<std::array<float, 3>, 3> intrinsicMatrix);
    ImgTransformation& setDistortionModel(CameraModel model);
    ImgTransformation& setDistortionCoefficients(std::vector<float> coefficients);

    /**
     * Remap a point from this transformation to another. If the intrinsics are different (e.g. different camera), the function will also use the
     * intrinsics to remap the point.
     * @param to Transformation to remap to
     * @param point Point to remap
     * @note This function assumes both transformations have the same source (eg. same source camera socket). If they don't, remapping will be inaccurate.
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
     * @param rect RotatedRect to remap
     */
    dai::RotatedRect remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const;

    /**
     * Project a 3D spatial point into 2D point in the current frame defined by this transformation.
     * @param point 3D point to project
     * @return Projected 2D point in the current frame
     * @note This function assumes that the point is in the coordinate system of the current frame.
     */
    dai::Point2f project3DPoint(const dai::Point3f& point) const;

    /**
     * Project a 2D point from the source frame defined by this transformation into a 2D point in the target frame defined by the to transformation. This
     * function will use the depth of the point to project it into 3D space and then reproject it back to 2D in the target frame.
     * @param to Target transformation to project to
     * @param point2f Source 2D point in the current frame
     * @param depth (mm) Depth of the point to project
     * @return Projected 2D point in the target frame (to transformation)
     */
    dai::Point2f projectPointTo(const ImgTransformation& to, dai::Point2f& point, float depth) const;

    /**
     * Project a 3D spatial point from the source coordinate system (this transformation) into a 2D point in the target frame (to transformation).
     * @param to Target transformation to project to
     * @param point3f 3D point to project
     * @return Projected 2D point in the target frame (to transformation)
     * @note This function assumes that the point3f is in the coordinate system of the current frame.
     */
    dai::Point2f project3DPointTo(const ImgTransformation& to, const dai::Point3f& point) const;

    /**
     * Project a 3D point from the source frame (from transformation) into a 2D point in the current frame (this transformation).
     * @param from Transformation to project from
     * @param point3f 3D point to project
     * @return Projected 2D point in the current frame
     * @note This function assumes that the point3f is in the coordinate system of the source frame.
     */
    dai::Point2f project3DPointFrom(const ImgTransformation& from, const dai::Point3f& point) const;

    /**
     * Remap a 3D point from the source coordinate system of this transformation to the coordinate system of the target transformation (to transformation).
     * @param to Target transformation to remap to
     * @param point 3D point to remap
     * @return Remapped 3D point in the target coordinate system
     * @note This function assumes that the point is in the coordinate system of the current frame.
     */
    dai::Point3f remap3DPointTo(const ImgTransformation& to, const dai::Point3f& point) const;

    /**
     * Remap a 3D point to the coordinate system of this transformation from the source coordinate system of the from transformation.
     * @param from Transformation to remap from
     * @param point 3D point to remap
     * @return Remapped 3D point in the current coordinate system
     * @note This function assumes that the point is in the coordinate system of the source frame.
     */
    dai::Point3f remap3DPointFrom(const ImgTransformation& from, const dai::Point3f& point) const;

    /**
     * Get the extrinsic transformation matrix from the source coordinate system of this transformation to the target coordinate system of the to
     * transformation.
     * @param to Target transformation to get extrinsics to
     * @return 4x4 homogeneous transformation matrix representing the extrinsics from this transformation to the target transformation
     * @note Both transformations must have a common toCameraSocket. Otherwise extrinsics cannot be calculated.
     */
    std::array<std::array<float, 4>, 4> getExtrinsicsTransformationMatrixTo(const ImgTransformation& to,
                                                                            bool useSpecTranslation = false,
                                                                            LengthUnit sourceUnit = LengthUnit::CENTIMETER) const;

    /**
     * Get the extrinsic rotation matrix from the source coordinate system of this transformation to the target coordinate system of the to transformation.
     * @param to Transformation to get extrinsics to
     * @return 3x3 rotation matrix representing the extrinsic rotation from this transformation to the target transformation
     */
    std::array<std::array<float, 3>, 3> getRotationMatrixTo(const ImgTransformation& to) const;

    /**
     * Get the extrinsic translation vector from the source coordinate system of this transformation to the target coordinate system of the to transformation.
     * @param to Transformation to get extrinsics to
     * @return 3x1 translation vector representing the extrinsic translation from this transformation to the target transformation
     */
    std::array<float, 3> getTranslationVectorTo(const ImgTransformation& to,
                                                bool useSpecTranslation = false,
                                                LengthUnit sourceUnit = LengthUnit::CENTIMETER) const;

    /**
     * Check if the transformations are aligned
     * @param to Transformation to compare with
     */
    bool isAlignedTo(const ImgTransformation& to) const;

    /**
     * Check if the transformations are valid. The transformations are valid if the source frame size and the current frame size are set.
     */
    bool isValid() const;

    DEPTHAI_SERIALIZE(ImgTransformation,
                      extrinsics,
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

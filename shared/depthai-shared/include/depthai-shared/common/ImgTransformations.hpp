#pragma once

#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/utility/Serialization.hpp"
#include "depthai-shared/utility/matrixOps.hpp"

namespace dai {
struct ImgTransformation {
    enum class Transformation : uint8_t {
        INIT,
        CROP,
        ROTATION,
        PAD,
        FLIP,
        SCALE,
    };
    Transformation transformationType = Transformation::INIT;

    // Crop parameters in absolute pixel values for the original image
    int topLeftCropX = 0, topLeftCropY = 0, bottomRightCropX = 0, bottomRightCropY = 0;

    // Crop parameters in absolute pixel values
    int topPadding = 0, bottomPadding = 0, leftPadding = 0, rightPadding = 0;

    // Used for flipping scale and rotation
    std::vector<std::vector<float>> transformationMatrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    // Precomput the inverse matrix for performance reasons
    std::vector<std::vector<float>> invTransformationMatrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    // Image size after transformation
    unsigned int afterTransformWidth = 0, afterTransformHeight = 0;

    // Image size before the transformation
    unsigned int beforeTransformWidth = 0, beforeTransformHeight = 0;

    static dai::Point2f transformPoint(ImgTransformation transformation, dai::Point2f point, bool& isClipped);

    static dai::Point2f invTransformPoint(ImgTransformation transformation, dai::Point2f point, bool& isClipped);

    static dai::Point2f clipPoint(dai::Point2f point, int imageWidth, int imageHeight, bool& isClipped);

   private:
    static dai::Point2f applyMatrixTransformation(dai::Point2f point, std::vector<std::vector<float>>& matrix);
};

DEPTHAI_SERIALIZE_EXT(ImgTransformation,
                      transformationType,
                      topLeftCropX,
                      topLeftCropY,
                      bottomRightCropX,
                      bottomRightCropY,
                      topPadding,
                      bottomPadding,
                      leftPadding,
                      rightPadding,
                      transformationMatrix,
                      afterTransformWidth,
                      afterTransformHeight,
                      beforeTransformWidth,
                      beforeTransformHeight);

class ImgTransformations {
   public:
    std::vector<ImgTransformation> transformations = {};

    bool invalidFlag = false;

    void invalidateTransformations();

    bool isInvalid() const;

    unsigned int getLastWidth() const;

    unsigned int getLastHeight() const;

    void addPadding(int topPadding, int bottomPadding, int leftPadding, int rightPadding);

    void addCrop(int topLeftCropX = 0, int topLeftCropY = 0, int bottomRightCropX = 0, int bottomRightCropY = 0);

    void addFlipVertical();

    void addFlipHorizontal();

    void addInitTransformation(int width, int height);

    void addRotation(float angle, dai::Point2f rotationPoint, int newWidth = 0, int newHeight = 0);

    void addScale(float scaleX, float scaleY);

    bool validateTransformationSizes() const;

    // API that is meant for performance reasons - so matrices can be precomputed.
    void addTransformation(std::vector<std::vector<float>> matrix,
                           std::vector<std::vector<float>> invMatrix,
                           ImgTransformation::Transformation transformation,
                           int newWidth,
                           int newHeight);

   private:
    ImgTransformation getNewTransformation() const;

    static std::vector<std::vector<float>> getFlipHorizontalMatrix(int width);

    static std::vector<std::vector<float>> getFlipVerticalMatrix(int height);

    static std::vector<std::vector<float>> getRotationMatrix(int px, int py, float theta);

    static std::vector<std::vector<float>> getScaleMatrix(float scaleX, float scaleY);
};

DEPTHAI_SERIALIZE_EXT(ImgTransformations, transformations, invalidFlag);

}  // namespace dai

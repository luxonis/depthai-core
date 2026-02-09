#include "depthai/common/ImgTransformations.hpp"

#include <assert.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <utility/matrixOps.hpp>
#include <vector>

#include "common/Point3f.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
#endif

#include "common/CameraModel.hpp"
#include "common/Point2f.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "pipeline/utilities/Alignment/AlignmentUtilities.hpp"

namespace dai {

constexpr float ROUND_UP_EPS = 1e-3f;

// Function to check if a point is inside a rotated rectangle
inline bool isPointInRotatedRectangle(const dai::Point2f& p, const dai::RotatedRect& rect) {
    auto theta = -rect.angle * (float)M_PI / 180.0f;
    float hw = rect.size.width / 2.0f;
    float hh = rect.size.height / 2.0f;
    float wvx = std::cos(theta);
    float wvy = std::sin(theta);
    float hvx = -std::sin(theta);
    float hvy = std::cos(theta);

    // Translate point relative to the center of rectangle
    float ptx = p.x - rect.center.x;
    float pty = p.y - rect.center.y;

    std::array<std::array<float, 2>, 2> basis = {{{wvx, hvx}, {wvy, hvy}}};

    // Rotate point to the rectangle's coordinate system
    auto transformed = impl::matvecmul(basis, {ptx, pty});

    return std::abs(transformed[0]) <= hw && std::abs(transformed[1]) <= hh;
}
inline bool RRinRR(const dai::RotatedRect& in, const dai::RotatedRect& out) {
    for(auto point : in.getPoints()) {
        if(!isPointInRotatedRectangle(point, out)) {
            return false;
        }
    }
    return true;
}

inline dai::Point3f transformPoint3f(const std::array<std::array<float, 4>, 4>& matrix, const dai::Point3f& point) {
    const float x = matrix[0][0] * point.x + matrix[0][1] * point.y + matrix[0][2] * point.z + matrix[0][3];
    const float y = matrix[1][0] * point.x + matrix[1][1] * point.y + matrix[1][2] * point.z + matrix[1][3];
    const float z = matrix[2][0] * point.x + matrix[2][1] * point.y + matrix[2][2] * point.z + matrix[2][3];
    const float w = matrix[3][0] * point.x + matrix[3][1] * point.y + matrix[3][2] * point.z + matrix[3][3];
    if(w != 0.0f && w != 1.0f) {
        return {x / w, y / w, z / w};
    }
    return {x, y, z};
}

std::array<float, 3> pixelToRay(const std::array<float, 3>& pxHomogeneous, const dai::ImgTransformation& transformation) {
    auto intrinsicMatrixInv = transformation.getSourceIntrinsicMatrixInv();
    auto distortionModel = transformation.getDistortionModel();
    auto distortionCoeffs = transformation.getDistortionCoefficients();

    std::array<float, 3> pxSensor = matrix::matVecMul(intrinsicMatrixInv, pxHomogeneous);
    std::array<float, 3> undistortedRay = undistortPoint(pxSensor, distortionModel, distortionCoeffs);
    std::array<float, 3> normalizedUndistortedRay = {undistortedRay[0] / undistortedRay[2], undistortedRay[1] / undistortedRay[2], 1.0f};
    return normalizedUndistortedRay;
}

std::array<float, 3> rayToPixel(const std::array<float, 3>& ray, const dai::ImgTransformation& transformation) {
    auto distortionModel = transformation.getDistortionModel();
    auto distortionCoeffs = transformation.getDistortionCoefficients();
    auto intrinsicMatrix = transformation.getSourceIntrinsicMatrix();

    std::array<float, 3> distortedRay = distortPoint(ray, distortionModel, distortionCoeffs);
    std::array<float, 3> pxHomogeneous = matrix::matVecMul(intrinsicMatrix, distortedRay);
    auto w = pxHomogeneous[2];
    return {pxHomogeneous[0] / w, pxHomogeneous[1] / w, 1.0f};
}

// Transforms a pixel coordinate inside the same source frame but different transformations (e.g. different distortion coefficients, crops, etc).
dai::Point2f interSourceFrameTransform(dai::Point2f sourcePt, const ImgTransformation& from, const ImgTransformation& to) {
    dai::Point2f transformed = sourcePt;
    if(from.isEqualTransformation(to)) {
        return transformed;
    }
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    transformed = opencvPointTransformation(sourcePt, from, to);
#else
    std::array<float, 3> pxHomogeneous = {sourcePt.x, sourcePt.y, 1.0f};

    // from px -> normalized undistorted ray
    std::array<float, 3> normalizedUndistortedRay = pixelToRay(pxHomogeneous, from);

    const auto transform = from.getExtrinsicsTransformationMatrixTo(to);

    const std::array<std::array<float, 3>, 3> extrinsicRotation = {{transform[0][0], transform[0][1], transform[0][2]},
                                                                   {transform[1][0], transform[1][1], transform[1][2]},
                                                                   {transform[2][0], transform[2][1], transform[2][2]}};

    const std::array<float, 3> rotatedRay = matrix::matVecMul(extrinsicRotation, normalizedUndistortedRay);
    // normalized undistorted ray -> to px
    std::array<float, 3> transformedPx = rayToPixel(rotatedRay, to);
    transformed = {transformedPx[0], transformedPx[1]};

// auto fromSource = from.getSourceIntrinsicMatrix();
// auto fromSourceInv = from.getSourceIntrinsicMatrixInv();
// auto toSource = to.getSourceIntrinsicMatrix();
// if(mateq(fromSource, toSource)) return sourcePt;
// auto transformMat = matmul(toSource, fromSourceInv);
// auto transformed = matvecmul(transformMat, {sourcePt.x, sourcePt.y});
#endif
    return transformed;
}
dai::RotatedRect interSourceFrameTransform(dai::RotatedRect sourceRect, const ImgTransformation& from, const ImgTransformation& to) {
    if(from.isEqualTransformation(to)) {
        return sourceRect;
    }
    const auto points = sourceRect.getPoints();
    std::vector<std::array<float, 2> > vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = interSourceFrameTransform(points[i], from, to);
        vPoints[i] = {point.x, point.y};
    }
    return impl::getOuterRotatedRect(vPoints);
}

void ImgTransformation::calcCrops() {
    if(cropsValid) return;
    const uint32_t inWidth = srcWidth;
    const uint32_t inHeight = srcHeight;
    // The following is done in ImageManip to get the source region of interest that is transformed
    size_t sourceMinX = 0;
    size_t sourceMaxX = inWidth;
    size_t sourceMinY = 0;
    size_t sourceMaxY = inHeight;
    for(const auto& crop : srcCrops) {
        auto corners = crop.getPoints();
        auto [minx, maxx, miny, maxy] =
            impl::getOuterRect({{corners[0].x, corners[0].y}, {corners[1].x, corners[1].y}, {corners[2].x, corners[2].y}, {corners[3].x, corners[3].y}});
        minx = std::max(minx, 0.0f);
        maxx = std::min(maxx, (float)inWidth);
        miny = std::max(miny, 0.0f);
        maxy = std::min(maxy, (float)inHeight);
        sourceMinX = std::max(sourceMinX, (size_t)std::floor(minx));
        sourceMinY = std::max(sourceMinY, (size_t)std::floor(miny));
        sourceMaxX = std::min(sourceMaxX, (size_t)std::ceil(maxx));
        sourceMaxY = std::min(sourceMaxY, (size_t)std::ceil(maxy));
    }
    srcCrop.center = {(float)(sourceMinX + sourceMaxX) / 2.0f, (float)(sourceMinY + sourceMaxY) / 2.0f};
    srcCrop.size = {(float)(sourceMaxX - sourceMinX), (float)(sourceMaxY - sourceMinY)};
    srcCrop.angle = 0.0f;
    // Calculate the destination crop from the source region of interest
    dstCrop = transformRect(srcCrop);
    cropsValid = true;
}

bool ImgTransformation::isEqualTransformation(const ImgTransformation& other) const {
    if(!matrix::mateq(getIntrinsicMatrix(), other.getIntrinsicMatrix())) return false;

    if(!matrix::mateq(getSourceIntrinsicMatrix(), other.getSourceIntrinsicMatrix())) return false;
    if(getDistortionModel() != other.getDistortionModel()) return false;
    if(getDistortionCoefficients() != other.getDistortionCoefficients()) return false;

    auto thisExtrinsics = getExtrinsics();
    auto otherExtrinsics = other.getExtrinsics();
    if(!thisExtrinsics.isEqualExtrinsics(otherExtrinsics)) return false;

    if(getSize() != other.getSize()) return false;
    if(getSourceSize() != other.getSourceSize()) return false;
    return true;
}

dai::Point2f ImgTransformation::transformPoint(dai::Point2f point) const {
    auto transformed = matrix::matVecMul(transformationMatrix, {point.x, point.y, 1});
    return {transformed[0] / transformed[2], transformed[1] / transformed[2]};
}
dai::RotatedRect ImgTransformation::transformRect(dai::RotatedRect rect) const {
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2> > vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = transformPoint(points[i]);
        vPoints[i] = {point.x, point.y};
    }
    return impl::getOuterRotatedRect(vPoints);
}
dai::Point2f ImgTransformation::invTransformPoint(dai::Point2f point) const {
    auto transformed = matrix::matVecMul(transformationMatrixInv, {point.x, point.y, 1});
    return {transformed[0] / transformed[2], transformed[1] / transformed[2]};
}
dai::RotatedRect ImgTransformation::invTransformRect(dai::RotatedRect rect) const {
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2> > vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = invTransformPoint(points[i]);
        vPoints[i] = {point.x, point.y};
    }
    return impl::getOuterRotatedRect(vPoints);
}

std::pair<size_t, size_t> ImgTransformation::getSize() const {
    return {width, height};
}
std::pair<size_t, size_t> ImgTransformation::getSourceSize() const {
    return {srcWidth, srcHeight};
}
std::array<std::array<float, 3>, 3> ImgTransformation::getMatrix() const {
    return transformationMatrix;
}
std::array<std::array<float, 3>, 3> ImgTransformation::getMatrixInv() const {
    return transformationMatrixInv;
}
std::array<std::array<float, 3>, 3> ImgTransformation::getSourceIntrinsicMatrix() const {
    return sourceIntrinsicMatrix;
}
std::array<std::array<float, 3>, 3> ImgTransformation::getSourceIntrinsicMatrixInv() const {
    return sourceIntrinsicMatrixInv;
}
std::array<std::array<float, 3>, 3> ImgTransformation::getIntrinsicMatrix() const {
    return matrix::matMul(transformationMatrix, sourceIntrinsicMatrix);
}
std::array<std::array<float, 3>, 3> ImgTransformation::getIntrinsicMatrixInv() const {
    return matrix::matMul(sourceIntrinsicMatrixInv, transformationMatrixInv);
}
float ImgTransformation::getDFov(bool source) const {
    float fovWidth = source ? srcWidth : width;
    float fovHeight = source ? srcHeight : height;
    if(fovHeight <= 0) {
        throw std::runtime_error(fmt::format("fovHeight is invalid. Height: {}", height));
    }
    if(fovWidth <= 0) {
        throw std::runtime_error(fmt::format("fovWidth is invalid. Width: {}", width));
    }
    float HFovDegrees = getHFov(source);

    // Calculate the diagonal ratio (DR)
    float dr = std::sqrt(std::pow(fovWidth, 2) + std::pow(fovHeight, 2));

    // Validate the horizontal FOV
    if(HFovDegrees <= 0 || HFovDegrees >= 180) {
        throw std::runtime_error(fmt::format("Horizontal FOV is invalid. Horizontal FOV: {}", HFovDegrees));
    }

    float HFovRadians = HFovDegrees * (static_cast<float>(M_PI) / 180.0f);

    // Calculate the tangent of half of the HFOV
    float tanHFovHalf = std::tan(HFovRadians / 2);

    // Calculate the tangent of half of the diagonal FOV
    float tanDiagonalFovHalf = (dr / fovWidth) * tanHFovHalf;

    // Calculate the diagonal FOV in radians
    float diagonalFovRadians = 2 * std::atan(tanDiagonalFovHalf);

    // Convert diagonal FOV to degrees
    float diagonalFovDegrees = diagonalFovRadians * (180.0f / static_cast<float>(M_PI));
    return diagonalFovDegrees;
}
float ImgTransformation::getHFov(bool source) const {
    float fx = source ? getSourceIntrinsicMatrix()[0][0] : getIntrinsicMatrix()[0][0];
    float fovWidth = source ? srcWidth : width;

    // Calculate horizontal FoV (in radians)
    float horizontalFoV = 2 * atan(fovWidth / (2.0f * fx));

    // Convert radians to degrees
    return horizontalFoV * 180.0f / (float)M_PI;
}
float ImgTransformation::getVFov(bool source) const {
    float fy = source ? getSourceIntrinsicMatrix()[1][1] : getIntrinsicMatrix()[1][1];
    float fovHeight = source ? srcHeight : height;

    // Calculate vertical FoV (in radians)
    float verticalFoV = 2 * atan(fovHeight / (2.0f * fy));

    // Convert radians to degrees
    return verticalFoV * 180.0f / (float)M_PI;
}
CameraModel ImgTransformation::getDistortionModel() const {
    return distortionModel;
}
std::vector<float> ImgTransformation::getDistortionCoefficients() const {
    return distortionCoefficients;
}
Extrinsics ImgTransformation::getExtrinsics() const {
    return extrinsics;
}
std::vector<dai::RotatedRect> ImgTransformation::getSrcCrops() const {
    return srcCrops;
}
bool ImgTransformation::getSrcMaskPt(size_t x, size_t y) {
    calcCrops();
    return isPointInRotatedRectangle({(float)x, (float)y}, srcCrop);
};
bool ImgTransformation::getDstMaskPt(size_t x, size_t y) {
    calcCrops();
    return isPointInRotatedRectangle({(float)x, (float)y}, dstCrop);
};

ImgTransformation& ImgTransformation::addTransformation(std::array<std::array<float, 3>, 3> matrix) {
    transformationMatrix = matrix::matMul(matrix, transformationMatrix);
    transformationMatrixInv = matrix::getMatrixInverse(transformationMatrix);
    cropsValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addCrop(int x, int y, int width, int height) {
    this->width = width;
    this->height = height;
    if(x != 0 || y != 0) {
        std::array<std::array<float, 3>, 3> cropMatrix = {{{1, 0, (float)-x}, {0, 1, (float)-y}, {0, 0, 1}}};
        addTransformation(cropMatrix);
    }
    std::array<std::array<float, 2>, 4> corners = {{{0, 0}, {(float)width, 0}, {(float)width, (float)height}, {0, (float)height}}};
    std::vector<std::array<float, 2> > srcCorners(4);
    for(auto i = 0; i < 4; ++i) {
        dai::Point2f transformed = invTransformPoint({corners[i][0], corners[i][1]});
        srcCorners[i] = {transformed.x, transformed.y};
    }
    auto rect = impl::getOuterRotatedRect(srcCorners);
    srcCrops.push_back(rect);
    cropsValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addPadding(int top, int bottom, int left, int right) {
    width += left + right;
    height += top + bottom;
    if(top != 0 || left != 0) {
        std::array<std::array<float, 3>, 3> padMatrix = {{{1, 0, (float)left}, {0, 1, (float)top}, {0, 0, 1}}};
        addTransformation(padMatrix);
    }
    cropsValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addFlipVertical() {
    std::array<std::array<float, 3>, 3> translateMatrix = {{{1, 0, -(width / 2.0f)}, {0, 1, -(height / 2.0f)}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> flipMatrix = {{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> translateMatrixInv = {{{1, 0, width / 2.0f}, {0, 1, height / 2.0f}, {0, 0, 1}}};
    addTransformation(translateMatrix);
    addTransformation(flipMatrix);
    addTransformation(translateMatrixInv);
    return *this;
}
ImgTransformation& ImgTransformation::addFlipHorizontal() {
    std::array<std::array<float, 3>, 3> translateMatrix = {{{1, 0, -(width / 2.0f)}, {0, 1, -(height / 2.0f)}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> flipMatrix = {{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> translateMatrixInv = {{{1, 0, width / 2.0f}, {0, 1, height / 2.0f}, {0, 0, 1}}};
    addTransformation(translateMatrix);
    addTransformation(flipMatrix);
    addTransformation(translateMatrixInv);
    return *this;
}
ImgTransformation& ImgTransformation::addRotation(float angle, dai::Point2f rotationPoint) {
    float angleRad = angle * (float)M_PI / 180.0f;
    if(rotationPoint.isNormalized()) {
        rotationPoint.x *= width;
        rotationPoint.y *= height;
        rotationPoint.normalized = false;
    }
    std::array<std::array<float, 3>, 3> translateMatrix = {{{1, 0, -rotationPoint.x}, {0, 1, -rotationPoint.y}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> rotateMatrix = {{{std::cos(angleRad), -std::sin(angleRad), 0}, {std::sin(angleRad), std::cos(angleRad), 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> translateMatrixInv = {{{1, 0, rotationPoint.x}, {0, 1, rotationPoint.y}, {0, 0, 1}}};
    addTransformation(translateMatrix);
    addTransformation(rotateMatrix);
    addTransformation(translateMatrixInv);
    return *this;
}
ImgTransformation& ImgTransformation::addScale(float scaleX, float scaleY) {
    width = width * scaleX + ROUND_UP_EPS;
    height = height * scaleY + ROUND_UP_EPS;
    std::array<std::array<float, 3>, 3> scaleMatrix = {{{scaleX, 0, 0}, {0, scaleY, 0}, {0, 0, 1}}};
    addTransformation(scaleMatrix);
    return *this;
}

ImgTransformation& ImgTransformation::addSrcCrops(const std::vector<dai::RotatedRect>& crops) {
    srcCrops.insert(srcCrops.end(), crops.begin(), crops.end());
    cropsValid = false;
    return *this;
}

ImgTransformation& ImgTransformation::setSize(size_t width, size_t height) {
    this->width = width;
    this->height = height;
    return *this;
}
ImgTransformation& ImgTransformation::setSourceSize(size_t width, size_t height) {
    this->srcWidth = width;
    this->srcHeight = height;
    return *this;
}
ImgTransformation& ImgTransformation::setIntrinsicMatrix(std::array<std::array<float, 3>, 3> intrinsicMatrix) {
    sourceIntrinsicMatrix = intrinsicMatrix;
    sourceIntrinsicMatrixInv = matrix::getMatrixInverse(intrinsicMatrix);
    return *this;
}
ImgTransformation& ImgTransformation::setExtrinsics(const Extrinsics& extrinsics) {
    this->extrinsics = extrinsics;
    return *this;
}
ImgTransformation& ImgTransformation::setDistortionModel(CameraModel model) {
    distortionModel = model;
    return *this;
}
ImgTransformation& ImgTransformation::setDistortionCoefficients(std::vector<float> coefficients) {
    distortionCoefficients = coefficients;
    return *this;
}

bool ImgTransformation::isValid() const {
    return srcWidth > 0 && srcHeight > 0 && width > 0 && height > 0;
}

dai::Point2f ImgTransformation::remapPointTo(const ImgTransformation& to, dai::Point2f point) const {
    bool normalized = point.isNormalized();
    if(normalized) {
        point.x *= width;
        point.y *= height;
        point.normalized = false;
    }
    // Assumes both transformations refer to the same extrinsics frame.
    // local frame -> sensor -> normalized ray -> sensor -> target local frame
    auto sourcePointFrom = invTransformPoint(point);                             // is this in px space in the sensor image plane
    auto sourcePointTo = interSourceFrameTransform(sourcePointFrom, *this, to);  // cast to ray then back into other image plane (eg. rectified / distorted)
    auto transformed = to.transformPoint(sourcePointTo);                         // back to pixel space in target frame
    if(normalized) {
        transformed.x /= to.width;
        transformed.y /= to.height;
        transformed.normalized = true;
    }
    return transformed;
}
dai::Point2f ImgTransformation::remapPointFrom(const ImgTransformation& from, dai::Point2f point) const {
    return from.remapPointTo(*this, point);
}

dai::RotatedRect ImgTransformation::remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const {
    bool normalized = rect.isNormalized();
    if(normalized) {
        rect = rect.denormalize(width, height);
    }
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2> > vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = remapPointTo(to, points[i]);
        vPoints[i] = {point.x, point.y};
    }
    auto transformed = impl::getOuterRotatedRect(vPoints);
    if(normalized) {
        transformed = transformed.normalize(to.width, to.height);
    }
    return transformed;
}
dai::RotatedRect ImgTransformation::remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const {
    return from.remapRectTo(*this, rect);
}

dai::Point2f ImgTransformation::project3DPoint(const dai::Point3f& point3f) const {
    const float x = point3f.x / point3f.z;
    const float y = point3f.y / point3f.z;
    const auto distorted = distortPoint({x, y, 1.0f}, distortionModel, distortionCoefficients);
    const auto projected = matrix::matVecMul(getIntrinsicMatrix(), distorted);
    return {projected[0] / projected[2], projected[1] / projected[2]};
}

dai::Point2f ImgTransformation::project3DPointTo(const ImgTransformation& to, const dai::Point3f& point) const {
    dai::Point3f remappedPoint = remap3DPointTo(to, point);
    return to.project3DPoint(remappedPoint);
}

dai::Point2f ImgTransformation::project3DPointFrom(const ImgTransformation& from, const dai::Point3f& point) const {
    return from.project3DPointTo(*this, point);
};

dai::Point3f ImgTransformation::remap3DPointTo(const ImgTransformation& to, const dai::Point3f& point) const {
    const auto transform = getExtrinsicsTransformationMatrixTo(to);
    return transformPoint3f(transform, point);
}

dai::Point3f ImgTransformation::remap3DPointFrom(const ImgTransformation& from, const dai::Point3f& point) const {
    const auto transform = from.getExtrinsicsTransformationMatrixTo(*this);
    return transformPoint3f(transform, point);
}

std::array<std::array<float, 4>, 4> ImgTransformation::getExtrinsicsTransformationMatrixTo(const ImgTransformation& to) const {
    // Calculate the extrinsics from this transformation to the target transformation
    // 1. Verify that the transformations have a common point
    if(to.extrinsics.toCameraSocket != extrinsics.toCameraSocket) {
        throw std::runtime_error("Cannot get extrinsics to a transformation with a different camera socket.");
    }

    // 2. Get the transformation matrices
    auto thisTransformationMatrix = extrinsics.getTransformationMatrix();                 // this -> Common
    auto toInverseTransformationMatrix = to.extrinsics.getInverseTransformationMatrix();  // inv(to -> Common) == Common -> to

    // 3. Multiply: (common->to) * (this -> Common)
    auto thisToTransformationMatrix = matrix::matMul(toInverseTransformationMatrix, thisTransformationMatrix);

    std::array<std::array<float, 4>, 4> result;
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            result[i][j] = thisToTransformationMatrix[i][j];
        }
    }
    return result;
}

bool ImgTransformation::isAlignedTo(const ImgTransformation& to) const {
    if(width != to.width || height != to.height) return false;
    if(this->distortionModel != to.distortionModel) return false;
    auto approxEqual = [](float a, float b, float absTol = ROUND_UP_EPS, float relTol = 2 * ROUND_UP_EPS) {
        return std::abs(a - b) <= (absTol + relTol * std::max(std::abs(a), std::abs(b)));
    };

    const size_t maxCoeffCount = std::max(distortionCoefficients.size(), to.distortionCoefficients.size());
    for(size_t i = 0; i < maxCoeffCount; ++i) {
        const float lhs = (i < distortionCoefficients.size()) ? distortionCoefficients[i] : 0.0f;
        const float rhs = (i < to.distortionCoefficients.size()) ? to.distortionCoefficients[i] : 0.0f;
        if(!approxEqual(lhs, rhs)) {
            return false;
        }
    }

    std::array<std::array<float, 3>, 3> thisIntrinsic = this->getIntrinsicMatrix();
    std::array<std::array<float, 3>, 3> toIntrinsic = to.getIntrinsicMatrix();
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            if(!approxEqual(toIntrinsic[i][j], thisIntrinsic[i][j])) return false;
        }
    }
    return true;
}

};  // namespace dai

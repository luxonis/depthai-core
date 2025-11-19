#include "depthai/common/ImgTransformations.hpp"

#include <assert.h>

#include <cstring>

#include "depthai/utility/ImageManipImpl.hpp"

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

inline std::array<std::array<float, 3>, 3> matmul(std::array<std::array<float, 3>, 3> A, std::array<std::array<float, 3>, 3> B) {
    return {{{A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
              A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
              A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]},
             {A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
              A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
              A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]},
             {A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
              A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
              A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]}}};
}

inline std::array<float, 2> matvecmul(std::array<std::array<float, 3>, 3> M, std::array<float, 2> vec) {
    auto x = M[0][0] * vec[0] + M[0][1] * vec[1] + M[0][2];
    auto y = M[1][0] * vec[0] + M[1][1] * vec[1] + M[1][2];
    auto z = M[2][0] * vec[0] + M[2][1] * vec[1] + M[2][2];
    return {x / z, y / z};
}

inline bool mateq(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B) {
    for(auto i = 0; i < 3; ++i)
        for(auto j = 0; j < 3; ++j)
            if(A[i][j] != B[i][j]) return false;
    return true;
}

std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix) {
    std::array<std::array<float, 3>, 3> inv;
    float det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
                - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
                + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    if(det == 0) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    std::array<std::array<float, 3>, 3> adj;

    adj[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
    adj[0][1] = -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]);
    adj[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]);

    adj[1][0] = -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]);
    adj[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]);
    adj[1][2] = -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]);

    adj[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    adj[2][1] = -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]);
    adj[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

    float invDet = 1.0f / det;

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            inv[i][j] = adj[i][j] * invDet;
        }
    }

    return inv;
}

dai::Point2f interSourceFrameTransform(dai::Point2f sourcePt, const ImgTransformation& from, const ImgTransformation& to) {
    auto fromSource = from.getSourceIntrinsicMatrix();
    auto fromSourceInv = from.getSourceIntrinsicMatrixInv();
    auto toSource = to.getSourceIntrinsicMatrix();
    if(mateq(fromSource, toSource)) return sourcePt;
    auto transformMat = matmul(toSource, fromSourceInv);
    auto transformed = matvecmul(transformMat, {sourcePt.x, sourcePt.y});
    return {transformed[0], transformed[1]};
}
dai::RotatedRect interSourceFrameTransform(dai::RotatedRect sourceRect, const ImgTransformation& from, const ImgTransformation& to) {
    auto fromSource = from.getSourceIntrinsicMatrix();
    auto toSource = to.getSourceIntrinsicMatrix();
    if(mateq(fromSource, toSource)) return sourceRect;

    const auto points = sourceRect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
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

dai::Point2f ImgTransformation::transformPoint(dai::Point2f point) const {
    auto transformed = matvecmul(transformationMatrix, {point.x, point.y});
    return {transformed[0], transformed[1]};
}
dai::RotatedRect ImgTransformation::transformRect(dai::RotatedRect rect) const {
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = transformPoint(points[i]);
        vPoints[i] = {point.x, point.y};
    }
    return impl::getOuterRotatedRect(vPoints);
}
dai::Point2f ImgTransformation::invTransformPoint(dai::Point2f point) const {
    auto transformed = matvecmul(transformationMatrixInv, {point.x, point.y});
    return {transformed[0], transformed[1]};
}
dai::RotatedRect ImgTransformation::invTransformRect(dai::RotatedRect rect) const {
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
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
    return matmul(transformationMatrix, sourceIntrinsicMatrix);
}
std::array<std::array<float, 3>, 3> ImgTransformation::getIntrinsicMatrixInv() const {
    return matmul(sourceIntrinsicMatrixInv, transformationMatrixInv);
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

    // Calculate the tangent of half of the VFOV
    float tanDiagonalFovHalf = (dr / fovWidth) * tanHFovHalf;

    // Calculate the VFOV in radians
    float diagonalFovRadians = 2 * std::atan(tanDiagonalFovHalf);

    // Convert VFOV to degrees
    float diagonalFovDegrees = diagonalFovRadians * (180.0f / static_cast<float>(M_PI));
    return diagonalFovDegrees;
}
float ImgTransformation::getHFov(bool source) const {
    float fx = source ? getSourceIntrinsicMatrix()[0][0] : getIntrinsicMatrix()[0][0];
    float fovWidth = source ? srcWidth : width;

    // Calculate vertical FoV (in radians)
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
    transformationMatrix = matmul(matrix, transformationMatrix);
    transformationMatrixInv = getMatrixInverse(transformationMatrix);
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
    std::vector<std::array<float, 2>> srcCorners(4);
    for(auto i = 0; i < 4; ++i) {
        srcCorners[i] = matvecmul(transformationMatrix, corners[i]);
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
    sourceIntrinsicMatrixInv = getMatrixInverse(intrinsicMatrix);
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
    auto sourcePointFrom = invTransformPoint(point);
    auto sourcePointTo = interSourceFrameTransform(sourcePointFrom, *this, to);
    auto transformed = to.transformPoint(sourcePointTo);
    if(normalized) {
        transformed.x /= to.width;
        transformed.y /= to.height;
        transformed.normalized = true;
    }
    return transformed;
}
dai::Point2f ImgTransformation::remapPointFrom(const ImgTransformation& from, dai::Point2f point) const {
    bool normalized = point.isNormalized();
    if(normalized) {
        point.x *= from.width;
        point.y *= from.height;
        point.normalized = false;
    }
    auto sourcePointFrom = from.invTransformPoint(point);
    auto sourcePointTo = interSourceFrameTransform(sourcePointFrom, from, *this);
    auto transformed = transformPoint(sourcePointTo);
    if(normalized) {
        transformed.x /= width;
        transformed.y /= height;
        transformed.normalized = true;
    }
    return transformed;
}
dai::RotatedRect ImgTransformation::remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const {
    bool normalized = rect.isNormalized();
    if(normalized) {
        rect = rect.denormalize(width, height);
    }
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
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
    bool normalized = rect.isNormalized();
    if(normalized) {
        rect = rect.denormalize(from.width, from.height);
    }
    const auto points = rect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = remapPointFrom(from, points[i]);
        vPoints[i] = {point.x, point.y};
    }
    auto transformed = impl::getOuterRotatedRect(vPoints);
    if(normalized) {
        transformed = transformed.normalize(width, height);
    }
    return transformed;
}

};  // namespace dai

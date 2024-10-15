#include "depthai/common/ImgTransformations.hpp"

#include <assert.h>

#include <cstring>
#include <sstream>

#include "common/RotatedRect.hpp"
#include "depthai/utility/ImageManipV2Impl.hpp"
#include "pipeline/datatype/ImageManipConfigV2.hpp"

namespace dai {

// Function to check if a point is inside a rotated rectangle
inline bool isPointInRotatedRectangle(const dai::Point2f& p, const dai::RotatedRect& rect) {
    // Step 1: Convert rotation to radians
    float theta = rect.angle * (float)M_PI / 180.0f;

    // Step 2: Calculate the difference between the point and the center of the rectangle
    float dx = p.x - rect.center.x;
    float dy = p.y - rect.center.y;

    // Step 3: Rotate the point to the rectangle's local coordinate system (inverse rotation)
    float localX = dx * std::cos(-theta) - dy * std::sin(-theta);
    float localY = dx * std::sin(-theta) + dy * std::cos(-theta);

    // Step 4: Check if the point lies within the bounds of the unrotated rectangle
    float halfWidth = rect.size.width / 2.0f;
    float halfHeight = rect.size.height / 2.0f;

    return std::abs(localX) <= halfWidth && std::abs(localY) <= halfHeight;
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
    for(auto i = 0; i < 3; ++i) for(auto j = 0; j < 3; ++j) if(A[i][j] != B[i][j]) return false;
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

    auto angleRad = sourceRect.angle * (float)M_PI / 180.0f;
    auto w = sourceRect.size.width;
    auto h = sourceRect.size.height;
    dai::Point2f widthVec = {std::cos(angleRad) * w, std::sin(angleRad) * w};
    dai::Point2f heightVec = {-std::sin(angleRad) * h, std::cos(angleRad) * h};
    dai::Point2f originVec = {0.f, 0.f};
    auto dstCenter = interSourceFrameTransform(sourceRect.center, from, to);
    auto dstWidthVec = interSourceFrameTransform(widthVec, from, to);
    auto dstHeightVec = interSourceFrameTransform(heightVec, from, to);
    auto dstOriginVec = interSourceFrameTransform(originVec, from, to);
    dstWidthVec.x -= dstOriginVec.x;
    dstWidthVec.y -= dstOriginVec.y;
    dstHeightVec.x -= dstOriginVec.x;
    dstHeightVec.y -= dstOriginVec.y;
    auto dstAngle = std::atan2(dstWidthVec.y, dstWidthVec.x) * 180.0f / (float)M_PI;
    return {dstCenter,
            {std::sqrt(dstWidthVec.x * dstWidthVec.x + dstWidthVec.y * dstWidthVec.y) * 2,
             std::sqrt(dstHeightVec.x * dstHeightVec.x + dstHeightVec.y * dstHeightVec.y) * 2},
            dstAngle};
}

void ImgTransformation::calcSrcBorder() {
    srcBorderComplex = false;
    srcMinX = srcMinY = 0;
    srcMaxX = width;
    srcMaxY = height;
    for(auto i = 0U; i < srcCrops.size(); ++i) {
        auto crop = srcCrops[i];
        const auto [minx, miny, maxx, maxy] = crop.getOuterRect();
        srcMinX = std::max(srcMinX, (size_t)roundf(minx));
        srcMinY = std::max(srcMinY, (size_t)roundf(miny));
        srcMaxX = std::min(srcMaxX, (size_t)roundf(maxx));
        srcMaxY = std::min(srcMaxY, (size_t)roundf(maxy));
        if(srcBorderComplex && crop.angle == 0.0f) {
            // If a simple crop is inside the complex border, it is no longer complex
            bool insideAll = true;
            for(auto j = 0U; j < i; ++j) {
                if(!RRinRR(crop, srcCrops[j])) {
                    insideAll = false;
                    break;
                }
            }
            if(insideAll) {
                srcBorderComplex = false;
            }
        } else if(!srcBorderComplex && crop.angle != 0.0f) {
            // If a complex crop is outside the simple border, it is still simple
            bool outsideAll = true;
            for(auto j = 0U; j < i; ++j) {
                if(!RRinRR(srcCrops[j], crop)) {
                    outsideAll = false;
                    break;
                }
            }
            if(!outsideAll) {
                srcBorderComplex = true;
            }
        }
    }
    srcMinX = std::max(srcMinX, (size_t)0U);
    srcMinY = std::max(srcMinY, (size_t)0U);
    srcMaxX = std::min(srcMaxX, width);
    srcMaxY = std::min(srcMaxY, height);
    srcBorderValid = true;
}
void ImgTransformation::calcDstBorder() {
    dstBorderComplex = false;
    srcMinX = srcMinY = 0;
    srcMaxX = width;
    srcMaxY = height;
    for(auto i = 0U; i < srcCrops.size(); ++i) {
        auto dstCrop = transformRect(srcCrops[i]);
        const auto [minx, miny, maxx, maxy] = dstCrop.getOuterRect();
        dstMinX = std::max(dstMinX, (size_t)roundf(minx));
        dstMinY = std::max(dstMinY, (size_t)roundf(miny));
        dstMaxX = std::min(dstMaxX, (size_t)roundf(maxx));
        dstMaxY = std::min(dstMaxY, (size_t)roundf(maxy));
        if(dstBorderComplex && std::abs(dstCrop.angle) < 1e-5f) {
            // If a simple crop is inside the complex border, it is no longer complex
            bool insideAll = true;
            for(auto j = 0U; j < i; ++j) {
                if(!RRinRR(dstCrop, transformRect(srcCrops[j]))) {
                    insideAll = false;
                    break;
                }
            }
            if(insideAll) {
                dstBorderComplex = false;
            }
        } else if(!dstBorderComplex && std::abs(dstCrop.angle) >= 1e-5f) {
            // If a complex crop is outside the simple border, it is still simple
            bool outsideAll = true;
            for(auto j = 0U; j < i; ++j) {
                if(!RRinRR(transformRect(srcCrops[j]), dstCrop)) {
                    outsideAll = false;
                    break;
                }
            }
            if(!outsideAll) {
                dstBorderComplex = true;
            }
        }
    }
    dstMinX = std::max(dstMinX, (size_t)0U);
    dstMinY = std::max(dstMinY, (size_t)0U);
    dstMaxX = std::min(dstMaxX, width);
    dstMaxY = std::min(dstMaxY, height);
    assert(dstMinX < dstMaxX && dstMinY < dstMaxY);
    dstBorderValid = true;
}

dai::Point2f ImgTransformation::transformPoint(dai::Point2f point) const {
    auto transformed = matvecmul(transformationMatrix, {point.x, point.y});
    return {transformed[0], transformed[1]};
}
dai::RotatedRect ImgTransformation::transformRect(dai::RotatedRect rect) const {
    auto angleRad = rect.angle * (float)M_PI / 180.0f;
    auto w = rect.size.width;
    auto h = rect.size.height;
    dai::Point2f widthVec = {std::cos(angleRad) * w, std::sin(angleRad) * w};
    dai::Point2f heightVec = {-std::sin(angleRad) * h, std::cos(angleRad) * h};
    dai::Point2f originVec = {0.f, 0.f};
    auto dstCenter = transformPoint(rect.center);
    auto dstWidthVec = transformPoint(widthVec);
    auto dstHeightVec = transformPoint(heightVec);
    auto dstOriginVec = transformPoint(originVec);
    dstWidthVec.x -= dstOriginVec.x;
    dstWidthVec.y -= dstOriginVec.y;
    dstHeightVec.x -= dstOriginVec.x;
    dstHeightVec.y -= dstOriginVec.y;
    auto dstAngle = std::atan2(dstWidthVec.y, dstWidthVec.x) * 180.0f / (float)M_PI;
    return {dstCenter,
            {std::sqrt(dstWidthVec.x * dstWidthVec.x + dstWidthVec.y * dstWidthVec.y) * 2,
             std::sqrt(dstHeightVec.x * dstHeightVec.x + dstHeightVec.y * dstHeightVec.y) * 2},
            dstAngle};
}
dai::Point2f ImgTransformation::invTransformPoint(dai::Point2f point) const {
    auto transformed = matvecmul(transformationMatrixInv, {point.x, point.y});
    return {transformed[0], transformed[1]};
}
dai::RotatedRect ImgTransformation::invTransformRect(dai::RotatedRect rect) const {
    auto angleRad = rect.angle * (float)M_PI / 180.0f;
    auto w = rect.size.width;
    auto h = rect.size.height;
    dai::Point2f widthVec = {std::cos(angleRad) * w, std::sin(angleRad) * w};
    dai::Point2f heightVec = {-std::sin(angleRad) * h, std::cos(angleRad) * h};
    dai::Point2f originVec = {0.f, 0.f};
    auto srcCenter = invTransformPoint(rect.center);
    auto srcWidthVec = invTransformPoint(widthVec);
    auto srcHeightVec = invTransformPoint(heightVec);
    auto srcOriginVec = invTransformPoint(originVec);
    srcWidthVec.x -= srcOriginVec.x;
    srcWidthVec.y -= srcOriginVec.y;
    srcHeightVec.x -= srcOriginVec.x;
    srcHeightVec.y -= srcOriginVec.y;
    auto srcAngle = std::atan2(srcWidthVec.y, srcWidthVec.x) * 180.0f / (float)M_PI;
    return {srcCenter,
            {std::sqrt(srcWidthVec.x * srcWidthVec.x + srcWidthVec.y * srcWidthVec.y) * 2,
             std::sqrt(srcHeightVec.x * srcHeightVec.x + srcHeightVec.y * srcHeightVec.y) * 2},
            srcAngle};
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
std::vector<dai::RotatedRect> ImgTransformation::getSrcCrops() const {
    return srcCrops;
}
bool ImgTransformation::getSrcMaskPt(size_t x, size_t y) {
    if(srcMaskValid) {
        return srcMask[y * width + x] != 0;
    }
    if(!srcBorderValid) {
        calcSrcBorder();
    }
    if(srcBorderComplex) {
        if(x < srcMinX || x >= srcMaxX || y < srcMinY || y >= srcMaxY) {
            return false;
        }
        for(auto crop : srcCrops) {
            if(!isPointInRotatedRectangle({(float)x, (float)y}, crop)) {
                return false;
            }
        }
        return true;
    } else {
        return x >= srcMinX && x < srcMaxX && y >= srcMinY && y < srcMaxY;
    }
};
bool ImgTransformation::getDstMaskPt(size_t x, size_t y) {
    if(dstMaskValid) {
        return dstMask[y * width + x] != 0;
    }
    if(!dstBorderValid) {
        calcDstBorder();
    }
    if(dstBorderComplex) {
        if(x < dstMinX || x >= dstMaxX || y < dstMinY || y >= dstMaxY) {
            return false;
        }
        auto ptSrc = invTransformPoint({(float)x, (float)y});
        for(auto crop : srcCrops) {
            if(!isPointInRotatedRectangle(ptSrc, crop)) {
                return false;
            }
        }
        return true;
    } else {
        return x >= dstMinX && x < dstMaxX && y >= dstMinY && y < dstMaxY;
    }
};
const std::vector<uint8_t>& ImgTransformation::getSrcMask(size_t srcWidth, size_t srcHeight) {
    if(srcMaskValid) {
        return srcMask;
    }
    if(!srcBorderValid) {
        calcSrcBorder();
    }
    srcMask.resize(srcWidth * srcHeight);
    memset(srcMask.data(), 0, srcMask.size());
    if(srcBorderComplex) {
        for(auto i = srcMinY; i < srcMaxY; ++i) {
            if(srcBorderComplex) {
                for(auto j = srcMinX; j < srcMaxX; ++j) {
                    srcMask[i * srcWidth + j] = 255;
                    for(auto crop : srcCrops) {
                        if(!isPointInRotatedRectangle({(float)j, (float)i}, crop)) {
                            srcMask[i * srcWidth + j] = 0;
                            break;
                        }
                    }
                }
            } else {
                memset(srcMask.data() + i * srcWidth + srcMinX, 255, srcMaxX - srcMinX);
            }
        }
    }
    srcMaskValid = true;
    return srcMask;
};
const std::vector<uint8_t>& ImgTransformation::getDstMask() {
    if(dstMaskValid) {
        return dstMask;
    }
    if(!dstBorderValid) {
        calcDstBorder();
    }
    dstMask.resize(width * height);
    memset(dstMask.data(), 0, dstMask.size());
    if(dstBorderComplex) {
        for(auto i = dstMinY; i < dstMaxY; ++i) {
            if(dstBorderComplex) {
                for(auto j = dstMinX; j < dstMaxX; ++j) {
                    dstMask[i * width + j] = 255;
                    for(auto crop : srcCrops) {
                        if(!isPointInRotatedRectangle({(float)j, (float)i}, transformRect(crop))) {
                            dstMask[i * width + j] = 0;
                            break;
                        }
                    }
                }
            } else {
                memset(dstMask.data() + i * width + dstMinX, 255, dstMaxX - dstMinX);
            }
        }
    }
    dstMaskValid = true;
    return dstMask;
};

ImgTransformation& ImgTransformation::addTransformation(std::array<std::array<float, 3>, 3> matrix) {
    transformationMatrix = matmul(matrix, transformationMatrix);
    transformationMatrixInv = getMatrixInverse(transformationMatrix);
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
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
    auto rect = impl::getRotatedRectFromPoints(srcCorners);
    srcCrops.push_back(rect);
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addPadding(int top, int bottom, int left, int right) {
    width += left + right;
    height += top + bottom;
    if(top != 0 || left != 0) {
        std::array<std::array<float, 3>, 3> padMatrix = {{{1, 0, (float)left}, {0, 1, (float)top}, {0, 0, 1}}};
        addTransformation(padMatrix);
    }
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addFlipVertical() {
    std::array<std::array<float, 3>, 3> translateMatrix = {{{1, 0, -(width / 2.0f)}, {0, 1, -(height / 2.0f)}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> flipMatrix = {{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> translateMatrixInv = {{{1, 0, width / 2.0f}, {0, 1, height / 2.0f}, {0, 0, 1}}};
    addTransformation(translateMatrix);
    addTransformation(flipMatrix);
    addTransformation(translateMatrixInv);
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addFlipHorizontal() {
    std::array<std::array<float, 3>, 3> translateMatrix = {{{1, 0, -(width / 2.0f)}, {0, 1, -(height / 2.0f)}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> flipMatrix = {{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> translateMatrixInv = {{{1, 0, width / 2.0f}, {0, 1, height / 2.0f}, {0, 0, 1}}};
    addTransformation(translateMatrix);
    addTransformation(flipMatrix);
    addTransformation(translateMatrixInv);
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
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
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}
ImgTransformation& ImgTransformation::addScale(float scaleX, float scaleY) {
    width *= scaleX;
    height *= scaleY;
    std::array<std::array<float, 3>, 3> scaleMatrix = {{{scaleX, 0, 0}, {0, scaleY, 0}, {0, 0, 1}}};
    addTransformation(scaleMatrix);
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}

ImgTransformation& ImgTransformation::addSrcCrops(const std::vector<dai::RotatedRect>& crops) {
    srcCrops.insert(srcCrops.end(), crops.begin(), crops.end());
    srcMaskValid = dstMaskValid = srcBorderValid = dstBorderValid = false;
    return *this;
}

bool ImgTransformation::isValid() const {
    return srcWidth > 0 && srcHeight > 0 && width > 0 && height > 0;
}

dai::Point2f ImgTransformation::remapPointTo(const ImgTransformation& to, dai::Point2f point) const {
    auto sourcePointFrom = invTransformPoint(point);
    auto sourcePointTo = interSourceFrameTransform(sourcePointFrom, *this, to);
    return to.transformPoint(sourcePointTo);
}
dai::Point2f ImgTransformation::remapPointTo(const std::array<std::array<float, 3>, 3>& to, dai::Point2f point) const {
    ImgTransformation toT(0, 0);
    toT.addTransformation(to);
    return remapPointTo(toT, point);
}
dai::Point2f ImgTransformation::remapPointFrom(const ImgTransformation& from, dai::Point2f point) const {
    auto sourcePointFrom = from.invTransformPoint(point);
    auto sourcePointTo = interSourceFrameTransform(sourcePointFrom, from, *this);
    return transformPoint(sourcePointTo);
}
dai::Point2f ImgTransformation::remapPointFrom(const std::array<std::array<float, 3>, 3>& from, dai::Point2f point) const {
    ImgTransformation fromT(0, 0);
    fromT.addTransformation(from);
    return remapPointFrom(fromT, point);
}
dai::RotatedRect ImgTransformation::remapRectTo(const ImgTransformation& to, dai::RotatedRect rect) const {
    return to.transformRect(invTransformRect(rect));
}
dai::RotatedRect ImgTransformation::remapRectTo(const std::array<std::array<float, 3>, 3>& to, dai::RotatedRect rect) const {
    ImgTransformation toT(0, 0);
    toT.addTransformation(to);
    return remapRectTo(toT, rect);
}
dai::RotatedRect ImgTransformation::remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const {
    return transformRect(from.invTransformRect(rect));
}
dai::RotatedRect ImgTransformation::remapRectFrom(const std::array<std::array<float, 3>, 3>& from, dai::RotatedRect rect) const {
    ImgTransformation fromT(0, 0);
    fromT.addTransformation(from);
    return remapRectFrom(fromT, rect);
}

std::string ImgTransformation::str() const {
    const int indent = 1;
    std::stringstream ss;
    auto doIndent = [&](int level) {
        for(auto i = 0; i < indent * level; ++i) ss << '\t';
    };
    ss << '\n';
    ss << "matrix: \n";
    for(auto i = 0; i < 3; ++i) {
        doIndent(1);
        ss << transformationMatrix[i][0];
        for(auto j = 1; j < 3; ++j) ss << '\t' << transformationMatrix[i][j];
        ss << '\n';
    }
    ss << "matrix inverse: \n";
    for(auto i = 0; i < 3; ++i) {
        doIndent(1);
        ss << transformationMatrixInv[i][0];
        for(auto j = 1; j < 3; ++j) ss << '\t' << transformationMatrixInv[i][j];
        ss << '\n';
    }
    ss << "intrinsics: \n";
    for(auto i = 0; i < 3; ++i) {
        doIndent(1);
        ss << sourceIntrinsicMatrix[i][0];
        for(auto j = 1; j < 3; ++j) ss << '\t' << sourceIntrinsicMatrix[i][j];
        ss << '\n';
    }
    ss << "intrinsics inverse: \n";
    for(auto i = 0; i < 3; ++i) {
        doIndent(1);
        ss << sourceIntrinsicMatrixInv[i][0];
        for(auto j = 1; j < 3; ++j) ss << '\t' << sourceIntrinsicMatrixInv[i][j];
        ss << '\n';
    }
    ss << "size: " << width << " x " << height << '\n';
    ss << "source size: " << srcWidth << " x " << srcHeight << '\n';
    return ss.str();
}

// std::vector<std::vector<float>> ImgTransformations::getFlipHorizontalMatrix(int width) {
//     auto scale = matrix::createScalingMatrix(-1, 1);
//     auto translate = matrix::createTranslationMatrix(width, 0);
//     return matrix::matMul(translate, scale);
// }
//
// std::vector<std::vector<float>> ImgTransformations::getFlipVerticalMatrix(int height) {
//     auto scale = matrix::createScalingMatrix(1, -1);
//     auto translate = matrix::createTranslationMatrix(0, height);
//     return matrix::matMul(translate, scale);
// }
//
// std::vector<std::vector<float>> ImgTransformations::getRotationMatrix(int px, int py, float theta) {
//     auto translateToOrigin = matrix::createTranslationMatrix(px, py);
//     auto rotate = matrix::createRotationMatrix(theta);
//     auto translateBack = matrix::createTranslationMatrix(-px, -py);
//
//     auto temp = matrix::matMul(translateToOrigin, rotate);
//     return matrix::matMul(temp, translateBack);
// }
//
// std::vector<std::vector<float>> ImgTransformations::getScaleMatrix(float scaleX, float scaleY) {
//     return matrix::createScalingMatrix(scaleX, scaleY);
// }
//
// unsigned int ImgTransformations::getLastHeight() const {
//     if(transformations.size() < 1) {
//         return 0;
//     }
//     return transformations.back().afterTransformHeight;
// }
//
// unsigned int ImgTransformations::getLastWidth() const {
//     if(transformations.size() < 1) {
//         return 0;
//     }
//     return transformations.back().afterTransformWidth;
// }
//
// void ImgTransformations::addPadding(int topPadding, int bottomPadding, int leftPadding, int rightPadding) {
//     auto paddingTransformation = getNewTransformation();
//     paddingTransformation.transformationType = ImgTransformation::Transformation::PAD;
//     paddingTransformation.afterTransformWidth = getLastWidth() + leftPadding + rightPadding;
//     paddingTransformation.afterTransformHeight = getLastHeight() + topPadding + bottomPadding;
//     paddingTransformation.topPadding = topPadding;
//     paddingTransformation.bottomPadding = bottomPadding;
//     paddingTransformation.leftPadding = leftPadding;
//     paddingTransformation.rightPadding = rightPadding;
//     transformations.push_back(paddingTransformation);
//     return;
// }
//
// void ImgTransformations::addCrop(int topLeftCropX, int topLeftCropY, int bottomRightCropX, int bottomRightCropY) {
//     if(transformations.size() < 1) {
//         throw std::runtime_error("Cannot set crop rotation without first setting an initial transformation");
//     }
//     auto croppingTransformation = getNewTransformation();
//     croppingTransformation.transformationType = ImgTransformation::Transformation::CROP;
//     croppingTransformation.afterTransformWidth = bottomRightCropX - topLeftCropX;
//     croppingTransformation.afterTransformHeight = bottomRightCropY - topLeftCropY;
//     croppingTransformation.topLeftCropX = topLeftCropX;
//     croppingTransformation.topLeftCropY = topLeftCropY;
//     croppingTransformation.bottomRightCropX = bottomRightCropX;
//     croppingTransformation.bottomRightCropY = bottomRightCropY;
//     transformations.push_back(croppingTransformation);
//     return;
// }
//
// void ImgTransformations::addScale(float scaleX, float scaleY) {
//     if(transformations.size() < 1) {
//         throw std::runtime_error("Cannot set scale rotation without first setting an initial transformation");
//     }
//     auto scaleTransformation = getNewTransformation();
//     scaleTransformation.transformationType = ImgTransformation::Transformation::SCALE;
//     scaleTransformation.afterTransformWidth = std::round(getLastWidth() * scaleX);
//     scaleTransformation.afterTransformHeight = std::round(getLastHeight() * scaleY);
//     scaleTransformation.transformationMatrix = getScaleMatrix(scaleX, scaleY);
//     scaleTransformation.invTransformationMatrix = std::vector<std::vector<float>>();
//     bool success = matrix::matInv(scaleTransformation.transformationMatrix, scaleTransformation.invTransformationMatrix);
//     if(!success) {
//         throw std::runtime_error("Could not invert matrix");
//     }
//     transformations.push_back(scaleTransformation);
//     return;
// }
//
// void ImgTransformations::addFlipVertical() {
//     if(transformations.size() < 1) {
//         throw std::runtime_error("Cannot set flip transformation image without first setting an initial transformation");
//     }
//     auto flipTransformation = getNewTransformation();
//     flipTransformation.transformationType = ImgTransformation::Transformation::FLIP;
//     flipTransformation.afterTransformWidth = getLastWidth();
//     flipTransformation.afterTransformHeight = getLastHeight();
//     flipTransformation.transformationMatrix = getFlipVerticalMatrix(getLastHeight());
//     flipTransformation.invTransformationMatrix = std::vector<std::vector<float>>();
//     bool success = matrix::matInv(flipTransformation.transformationMatrix, flipTransformation.invTransformationMatrix);
//     if(!success) {
//         throw std::runtime_error("Could not invert matrix");
//     }
//     transformations.push_back(flipTransformation);
//     return;
// }
//
// void ImgTransformations::addFlipHorizontal() {
//     if(transformations.size() < 1) {
//         throw std::runtime_error("Cannot set flip transformation without first setting an initial transformation");
//     }
//     auto flipTransformation = getNewTransformation();
//     flipTransformation.transformationType = ImgTransformation::Transformation::FLIP;
//     flipTransformation.afterTransformWidth = getLastWidth();
//     flipTransformation.afterTransformHeight = getLastHeight();
//     flipTransformation.transformationMatrix = getFlipHorizontalMatrix(getLastWidth());
//     flipTransformation.invTransformationMatrix = std::vector<std::vector<float>>();
//     bool success = matrix::matInv(flipTransformation.transformationMatrix, flipTransformation.invTransformationMatrix);
//     if(!success) {
//         throw std::runtime_error("Could not invert matrix");
//     }
//     transformations.push_back(flipTransformation);
//     return;
// }
//
// void ImgTransformations::addInitTransformation(int width, int height) {
//     if(transformations.size() > 0) {
//         throw std::runtime_error("Cannot set initial transformation after other transformations have been set");
//     }
//     ImgTransformation initTransformation;
//     initTransformation.transformationType = ImgTransformation::Transformation::INIT;
//     initTransformation.afterTransformWidth = width;
//     initTransformation.afterTransformHeight = height;
//     initTransformation.beforeTransformWidth = width;
//     initTransformation.beforeTransformHeight = height;
//     transformations.push_back(initTransformation);
//     return;
// }
//
// void ImgTransformations::addRotation(float angle, dai::Point2f rotationPoint, int newWidth, int newHeight) {
//     if(transformations.size() < 1) {
//         throw std::runtime_error("Cannot set rotation transformation without first setting an initial transformation");
//     }
//     auto rotationTransformation = getNewTransformation();
//     rotationTransformation.transformationType = ImgTransformation::Transformation::ROTATION;
//     rotationTransformation.afterTransformWidth = newWidth;
//     rotationTransformation.afterTransformHeight = newHeight;
//     rotationTransformation.transformationMatrix = getRotationMatrix(rotationPoint.x, rotationPoint.y, angle);
//     rotationTransformation.invTransformationMatrix = getRotationMatrix(rotationPoint.x, rotationPoint.y, -angle);
//     transformations.push_back(rotationTransformation);
// }
//
// // API that is meant for performance reasons - so matrices can be precomputed.
// void ImgTransformations::addTransformation(std::vector<std::vector<float>> matrix,
//                                            std::vector<std::vector<float>> invMatrix,
//                                            ImgTransformation::Transformation transformation,
//                                            int newWidth,
//                                            int newHeight) {
//     auto imgTransformation = getNewTransformation();
//     imgTransformation.transformationType = transformation;
//     imgTransformation.afterTransformWidth = newWidth;
//     imgTransformation.afterTransformHeight = newHeight;
//     imgTransformation.transformationMatrix = matrix;
//     imgTransformation.invTransformationMatrix = invMatrix;
//     return;
// }
//
// dai::Point2f ImgTransformation::applyMatrixTransformation(dai::Point2f point, std::vector<std::vector<float>>& matrix) {
//     if(point.isNormalized() && point.x != 0.0f && point.y != 0.0f) {
//         throw std::runtime_error("Cannot apply matrix transformation to normalized point (x = " + std::to_string(point.x) + ", y = " +
//         std::to_string(point.y)
//                                  + ")");
//     }
//     std::vector<float> homogenousPoint = {point.x, point.y, 1.0f};
//     std::vector<float> transformedPoint = {0.0f, 0.0f, 0.0f};
//
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 3; j++) {
//             transformedPoint[i] += matrix[i][j] * homogenousPoint[j];
//         }
//     }
//     if(transformedPoint[2] == 0) {
//         throw std::runtime_error("Homogeneous coordinate is zero");
//     }
//     return Point2f(std::round(transformedPoint[0] / transformedPoint[2]), std::round(transformedPoint[1] / transformedPoint[2]));
// }
//
// dai::Point2f ImgTransformation::clipPoint(dai::Point2f point, int imageWidth, int imageHeight, bool& isClipped) {
//     if(imageHeight == 0 && imageWidth == 0) {
//         throw std::runtime_error("Image width and height must be greater than zero");
//     }
//     isClipped = false;
//     if(point.x < 0) {
//         point.x = 0;
//         isClipped = true;
//     }
//     if(point.y < 0) {
//         point.y = 0;
//         isClipped = true;
//     }
//     if(point.x > imageWidth) {
//         point.x = imageWidth;
//         isClipped = true;
//     }
//     if(point.y > imageHeight) {
//         point.y = imageHeight;
//         isClipped = true;
//     }
//     return point;
// }
//
// dai::Point2f ImgTransformation::transformPoint(ImgTransformation transformation, dai::Point2f point, bool& isClipped) {
//     switch(transformation.transformationType) {
//         case ImgTransformation::Transformation::INIT:
//             break;
//         case ImgTransformation::Transformation::PAD:
//             point.x = point.x + transformation.leftPadding;
//             point.y = point.y + transformation.topPadding;
//             break;
//         case ImgTransformation::Transformation::CROP:
//             point.x = point.x - transformation.topLeftCropX;
//             point.y = point.y - transformation.topLeftCropY;
//             break;
//         case ImgTransformation::Transformation::ROTATION:
//         case ImgTransformation::Transformation::FLIP:
//         case ImgTransformation::Transformation::SCALE:
//             point = applyMatrixTransformation(point, transformation.transformationMatrix);
//             break;
//         default:
//             break;
//     }
//     point = clipPoint(point, transformation.afterTransformWidth, transformation.afterTransformHeight, isClipped);
//     return point;
// }
//
// dai::Point2f ImgTransformation::invTransformPoint(ImgTransformation transformation, dai::Point2f point, bool& isClipped) {
//     switch(transformation.transformationType) {
//         case ImgTransformation::Transformation::PAD:
//             point.x = point.x - transformation.leftPadding;
//             point.y = point.y - transformation.topPadding;
//             break;
//         case ImgTransformation::Transformation::INIT:
//             break;
//         case ImgTransformation::Transformation::CROP:
//             point.x = point.x + transformation.topLeftCropX;
//             point.y = point.y + transformation.topLeftCropY;
//             break;
//         case ImgTransformation::Transformation::ROTATION:
//         case ImgTransformation::Transformation::FLIP:
//         case ImgTransformation::Transformation::SCALE:
//             point = applyMatrixTransformation(point, transformation.invTransformationMatrix);
//             break;
//         default:
//             break;
//     }
//     point = clipPoint(point, transformation.beforeTransformWidth, transformation.beforeTransformHeight, isClipped);
//     return point;
// }
//
// bool ImgTransformations::validateTransformationSizes() const {
//     if(transformations.size() < 1) {
//         return true;
//     }
//
//     auto lastWidth = transformations[0].beforeTransformWidth;
//     auto lastHeight = transformations[0].beforeTransformHeight;
//     for(auto& transformation : transformations) {
//         if(transformation.beforeTransformWidth != lastWidth || transformation.beforeTransformHeight != lastHeight) {
//             return false;
//         }
//         lastWidth = transformation.afterTransformWidth;
//         lastHeight = transformation.afterTransformHeight;
//     }
//     return true;
// }
//
// ImgTransformation ImgTransformations::getNewTransformation() const {
//     ImgTransformation transformation;
//     transformation.beforeTransformWidth = getLastWidth();
//     transformation.beforeTransformHeight = getLastHeight();
//     return transformation;
// }
//
// void ImgTransformations::invalidateTransformations() {
//     invalidFlag = true;
// }
//
// bool ImgTransformations::isInvalid() const {
//     return invalidFlag || !validateTransformationSizes();
// }

};  // namespace dai

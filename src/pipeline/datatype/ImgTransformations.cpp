#include "depthai/common/ImgTransformations.hpp"

#include <assert.h>

#include <algorithm>
#include <cstring>
#include <sstream>

#include "depthai/utility/ImageManipV2Impl.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"

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

    const auto points = sourceRect.getPoints();
    std::vector<std::array<float, 2>> vPoints(points.size());
    for(auto i = 0U; i < points.size(); ++i) {
        auto point = interSourceFrameTransform(points[i], from, to);
        vPoints[i] = {point.x, point.y};
    }
    return impl::getRotatedRectFromPoints(vPoints);
}

// void ImgTransformation::calcSrcBorder() {
//     srcBorderComplex = false;
//     srcMinX = srcMinY = 0;
//     srcMaxX = width;
//     srcMaxY = height;
//     for(auto i = 0U; i < srcCrops.size(); ++i) {
//         auto crop = srcCrops[i];
//         const auto [minx, miny, maxx, maxy] = crop.getOuterRect();
//         srcMinX = std::max(srcMinX, (size_t)roundf(minx));
//         srcMinY = std::max(srcMinY, (size_t)roundf(miny));
//         srcMaxX = std::min(srcMaxX, (size_t)roundf(maxx));
//         srcMaxY = std::min(srcMaxY, (size_t)roundf(maxy));
//         if(srcBorderComplex && crop.angle == 0.0f) {
//             // If a simple crop is inside the complex border, it is no longer complex
//             bool insideAll = true;
//             for(auto j = 0U; j < i; ++j) {
//                 if(!RRinRR(crop, srcCrops[j])) {
//                     insideAll = false;
//                     break;
//                 }
//             }
//             if(insideAll) {
//                 srcBorderComplex = false;
//             }
//         } else if(!srcBorderComplex && crop.angle != 0.0f) {
//             // If a complex crop is outside the simple border, it is still simple
//             bool outsideAll = true;
//             for(auto j = 0U; j < i; ++j) {
//                 if(!RRinRR(srcCrops[j], crop)) {
//                     outsideAll = false;
//                     break;
//                 }
//             }
//             if(!outsideAll) {
//                 srcBorderComplex = true;
//             }
//         }
//     }
//     srcMinX = std::max(srcMinX, (size_t)0U);
//     srcMinY = std::max(srcMinY, (size_t)0U);
//     srcMaxX = std::min(srcMaxX, width);
//     srcMaxY = std::min(srcMaxY, height);
//     srcBorderValid = true;
// }
// void ImgTransformation::calcDstBorder() {
//     dstBorderComplex = false;
//     bool first = true;
//     for(auto i = 0U; i < srcCrops.size(); ++i) {
//         auto dstCrop = transformRect(srcCrops[i]);
//         const auto [minx, miny, maxx, maxy] = dstCrop.getOuterRect();
//         if(first) {
//             dstMinX = (size_t)roundf(minx);
//             dstMinY = (size_t)roundf(miny);
//             dstMaxX = (size_t)roundf(maxx);
//             dstMaxY = (size_t)roundf(maxy);
//             first = false;
//         } else {
//             dstMinX = std::max(dstMinX, (size_t)roundf(minx));
//             dstMinY = std::max(dstMinY, (size_t)roundf(miny));
//             dstMaxX = std::min(dstMaxX, (size_t)roundf(maxx));
//             dstMaxY = std::min(dstMaxY, (size_t)roundf(maxy));
//         }
//         if(dstBorderComplex && std::abs(dstCrop.angle) < 1e-5f) {
//             // If a simple crop is inside the complex border, it is no longer complex
//             bool insideAll = true;
//             for(auto j = 0U; j < i; ++j) {
//                 if(!RRinRR(dstCrop, transformRect(srcCrops[j]))) {
//                     insideAll = false;
//                     break;
//                 }
//             }
//             if(insideAll) {
//                 dstBorderComplex = false;
//             }
//         } else if(!dstBorderComplex && std::abs(dstCrop.angle) >= 1e-5f) {
//             // If a complex crop is outside the simple border, it is still simple
//             bool outsideAll = true;
//             for(auto j = 0U; j < i; ++j) {
//                 if(!RRinRR(transformRect(srcCrops[j]), dstCrop)) {
//                     outsideAll = false;
//                     break;
//                 }
//             }
//             if(!outsideAll) {
//                 dstBorderComplex = true;
//             }
//         }
//     }
//     dstMinX = std::max(dstMinX, (size_t)0U);
//     dstMinY = std::max(dstMinY, (size_t)0U);
//     dstMaxX = std::min(dstMaxX, width);
//     dstMaxY = std::min(dstMaxY, height);
//     assert(dstMinX < dstMaxX && dstMinY < dstMaxY);
//     dstBorderValid = true;
// }

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
    return impl::getRotatedRectFromPoints(vPoints);
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
    return impl::getRotatedRectFromPoints(vPoints);
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
// bool ImgTransformation::getSrcMaskPt(size_t x, size_t y) {
//     if(srcMaskValid) {
//         return srcMask[y * width + x] != 0;
//     }
//     if(!srcBorderValid) {
//         calcSrcBorder();
//     }
//     if(srcBorderComplex) {
//         if(x < srcMinX || x >= srcMaxX || y < srcMinY || y >= srcMaxY) {
//             return false;
//         }
//         for(auto crop : srcCrops) {
//             if(!isPointInRotatedRectangle({(float)x, (float)y}, crop)) {
//                 return false;
//             }
//         }
//         return true;
//     } else {
//         return x >= srcMinX && x < srcMaxX && y >= srcMinY && y < srcMaxY;
//     }
// };
// bool ImgTransformation::getDstMaskPt(size_t x, size_t y) {
//     if(dstMaskValid) {
//         return dstMask[y * width + x] != 0;
//     }
//     if(!dstBorderValid) {
//         calcDstBorder();
//     }
//     if(dstBorderComplex) {
//         if(x < dstMinX || x >= dstMaxX || y < dstMinY || y >= dstMaxY) {
//             return false;
//         }
//         auto ptSrc = invTransformPoint({(float)x, (float)y});
//         for(auto crop : srcCrops) {
//             if(!isPointInRotatedRectangle(ptSrc, crop)) {
//                 return false;
//             }
//         }
//         return true;
//     } else {
//         return x >= dstMinX && x < dstMaxX && y >= dstMinY && y < dstMaxY;
//     }
// };
// const std::vector<uint8_t>& ImgTransformation::getSrcMask(size_t srcWidth, size_t srcHeight) {
//     if(srcMaskValid) {
//         return srcMask;
//     }
//     if(!srcBorderValid) {
//         calcSrcBorder();
//     }
//     srcMask.resize(srcWidth * srcHeight);
//     memset(srcMask.data(), 0, srcMask.size());
//     if(srcBorderComplex) {
//         for(auto i = srcMinY; i < srcMaxY; ++i) {
//             if(srcBorderComplex) {
//                 for(auto j = srcMinX; j < srcMaxX; ++j) {
//                     srcMask[i * srcWidth + j] = 255;
//                     for(auto crop : srcCrops) {
//                         if(!isPointInRotatedRectangle({(float)j, (float)i}, crop)) {
//                             srcMask[i * srcWidth + j] = 0;
//                             break;
//                         }
//                     }
//                 }
//             } else {
//                 memset(srcMask.data() + i * srcWidth + srcMinX, 255, srcMaxX - srcMinX);
//             }
//         }
//     }
//     srcMaskValid = true;
//     return srcMask;
// };
// const std::vector<uint8_t>& ImgTransformation::getDstMask() {
//     if(dstMaskValid) {
//         return dstMask;
//     }
//     if(!dstBorderValid) {
//         calcDstBorder();
//     }
//     dstMask.resize(width * height);
//     memset(dstMask.data(), 0, dstMask.size());
//     if(dstBorderComplex) {
//         for(auto i = dstMinY; i < dstMaxY; ++i) {
//             if(dstBorderComplex) {
//                 for(auto j = dstMinX; j < dstMaxX; ++j) {
//                     dstMask[i * width + j] = 255;
//                     for(auto crop : srcCrops) {
//                         if(!isPointInRotatedRectangle({(float)j, (float)i}, transformRect(crop))) {
//                             dstMask[i * width + j] = 0;
//                             break;
//                         }
//                     }
//                 }
//             } else {
//                 memset(dstMask.data() + i * width + dstMinX, 255, dstMaxX - dstMinX);
//             }
//         }
//     }
//     dstMaskValid = true;
//     return dstMask;
// };

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
    auto sourceRectFrom = invTransformRect(rect);
    auto sourceRectTo = interSourceFrameTransform(sourceRectFrom, *this, to);
    return to.transformRect(sourceRectTo);
}
dai::RotatedRect ImgTransformation::remapRectTo(const std::array<std::array<float, 3>, 3>& to, dai::RotatedRect rect) const {
    ImgTransformation toT(0, 0);
    toT.addTransformation(to);
    return remapRectTo(toT, rect);
}
dai::RotatedRect ImgTransformation::remapRectFrom(const ImgTransformation& from, dai::RotatedRect rect) const {
    auto sourceRectFrom = from.invTransformRect(rect);
    auto sourceRectTo = interSourceFrameTransform(sourceRectFrom, from, *this);
    return transformRect(sourceRectTo);
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

};  // namespace dai

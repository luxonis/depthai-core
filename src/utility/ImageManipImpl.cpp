#include "depthai/utility/ImageManipImpl.hpp"

#include <stdexcept>

#include "OCVPorts.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
#endif

#if defined(WIN32) || defined(_WIN32)
    #define _RESTRICT
#else
    #define _RESTRICT __restrict__
#endif

void dai::impl::transformOpenCV(const uint8_t* src,
                                uint8_t* dst,
                                const size_t srcWidth,
                                const size_t srcHeight,
                                const size_t srcStride,
                                const size_t dstWidth,
                                const size_t dstHeight,
                                const size_t dstStride,
                                const uint16_t numChannels,
                                const uint16_t bpp,
                                const std::array<std::array<float, 3>, 3> matrix,
                                const std::vector<uint32_t>& background,
                                const FrameSpecs& srcImgSpecs,
                                const size_t sourceMinX,
                                const size_t sourceMinY,
                                const size_t sourceMaxX,
                                const size_t sourceMaxY) {
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_OPENCV
    auto type = CV_8UC1;
    switch(numChannels) {
        case 1:
            switch(bpp) {
                case 1:
                    type = CV_8UC1;
                    break;
                case 2:
                    type = CV_16UC1;
                    break;
                default:
                    assert(false);
            }
            break;
        case 2:
            assert(bpp == 1);
            type = CV_8UC2;
            break;
        case 3:
            assert(bpp == 1);
            type = CV_8UC3;
            break;
        default:
            assert(false);
    }
    auto bg = numChannels == 1 ? cv::Scalar(background[0])
                               : (numChannels == 2 ? cv::Scalar(background[0], background[1]) : cv::Scalar(background[0], background[1], background[2]));
    const cv::Mat cvSrc(srcHeight, srcWidth, type, const_cast<uint8_t*>(src), srcStride);
    cv::Mat cvDst(dstHeight, dstWidth, type, dst, dstStride);
    int ssF = srcImgSpecs.width / srcWidth;
    assert(ssF == (int)(srcImgSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));  // Sanity check
    if(floatEq(matrix[2][0], 0) && floatEq(matrix[2][1], 0) && floatEq(matrix[2][2], 1)) {
        // Affine transform
        float affine[6] = {matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF};
        cv::Rect roi(sourceMinX / ssF, sourceMinY / ssF, (sourceMaxX - sourceMinX) / ssF, (sourceMaxY - sourceMinY) / ssF);
        if(sourceMinX != 0 || sourceMinY != 0) {
            affine[2] = affine[0] * ((float)sourceMinX / ssF) + affine[1] * ((float)sourceMinY / ssF) + affine[2];
            affine[5] = affine[3] * ((float)sourceMinX / ssF) + affine[4] * ((float)sourceMinY / ssF) + affine[5];
        }
        if(floatEq(affine[0], 1.f) && floatEq(affine[1], 0.f) && floatEq(affine[3], 0.f) && floatEq(affine[4], 1.f) && floatEq(affine[5], 0.f)) {
            // Crop only
            cvSrc(roi).copyTo(cvDst);
        } else {
            cv::Mat cvAffine(2, 3, CV_32F, affine);
            cv::warpAffine(cvSrc(roi),
                           cvDst,
                           cvAffine,
                           cv::Size(dstWidth, dstHeight),
                           cv::INTER_LINEAR,
                           cv::BORDER_CONSTANT,
                           bg);  // TODO(asahtik): Add support for different border types
        }
    } else {
        // Perspective transform
        float projection[9] = {
            matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF, matrix[2][0], matrix[2][1], matrix[2][2]};
        cv::Rect roi(sourceMinX / ssF, sourceMinY / ssF, (sourceMaxX - sourceMinX) / ssF, (sourceMaxY - sourceMinY) / ssF);
        if(sourceMinX != 0 || sourceMinY != 0) {
            projection[2] = projection[0] * ((float)sourceMinX / ssF) + projection[1] * ((float)sourceMinY / ssF) + projection[2];
            projection[5] = projection[3] * ((float)sourceMinX / ssF) + projection[4] * ((float)sourceMinY / ssF) + projection[5];
            projection[8] = projection[6] * ((float)sourceMinX / ssF) + projection[7] * ((float)sourceMinY / ssF) + projection[8];
        }
        cv::Mat cvProjection(3, 3, CV_32F, projection);
        cv::warpPerspective(cvSrc(roi),
                            cvDst,
                            cvProjection,
                            cv::Size(dstWidth, dstHeight),
                            cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT,
                            bg);  // TODO(asahtik): Add support for different border types
    }
#else
    (void)(src);
    (void)(dst);
    (void)(srcWidth);
    (void)(srcHeight);
    (void)(srcStride);
    (void)(dstWidth);
    (void)(dstHeight);
    (void)(dstStride);
    (void)(numChannels);
    (void)(bpp);
    (void)(matrix);
    (void)(background);
    (void)(srcImgSpecs);
    (void)(sourceMinX);
    (void)(sourceMinY);
    (void)(sourceMaxX);
    (void)(sourceMaxY);
#endif
}
void dai::impl::transformFastCV(const uint8_t* src,
                                uint8_t* dst,
                                const size_t srcWidth,
                                const size_t srcHeight,
                                const size_t srcStride,
                                const size_t dstWidth,
                                const size_t dstHeight,
                                const size_t dstStride,
                                const uint16_t numChannels,
                                const uint16_t bpp,
                                const std::array<std::array<float, 3>, 3> matrix,
                                const std::vector<uint32_t>& background,
                                const FrameSpecs& srcImgSpecs,
                                const size_t sourceMinX,
                                const size_t sourceMinY,
                                const size_t sourceMaxX,
                                const size_t sourceMaxY,
                                uint32_t* fastCvBorder) {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
    if(numChannels != 3 && numChannels != 1) throw std::runtime_error("Only 1 or 3 channels supported with FastCV");
    if(bpp != 1) throw std::runtime_error("Only 8bpp supported with FastCV");
    if(!((ptrdiff_t)src % 128 == 0 && (ptrdiff_t)dst % 128 == 0 && (ptrdiff_t)fastCvBorder % 128 == 0 && srcStride % 8 == 0 && srcStride > 0)) {
        throw std::runtime_error("Assumptions not taken into account");
    }
    int ssF = srcSpecs.width / srcWidth;
    assert(ssF == (int)(srcSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));  // Sanity check
    if(floatEq(matrix[2][0], 0) && floatEq(matrix[2][1], 0) && floatEq(matrix[2][2], 1)) {
        // Affine transform
        float affine[6] = {matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF};
        if(isSingleChannelu8(src)) {
            fcvTransformAffineClippedu8_v3(src,
                                           srcWidth,
                                           srcHeight,
                                           srcStride,
                                           affine,
                                           dst,
                                           dstWidth,
                                           dstHeight,
                                           dstStride,
                                           nullptr,
                                           FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                           FASTCV_BORDER_CONSTANT,
                                           0);
        } else {
            fcv3ChannelTransformAffineClippedBCu8(src, srcWidth, srcHeight, srcStride, affine, dst, dstWidth, dstHeight, dstStride, fastCvBorder);
        }
    } else {
        // Perspective transform
        float projection[9] = {
            matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF, matrix[2][0], matrix[2][1], matrix[2][2]};
        fcvStatus status = fcvStatus::FASTCV_SUCCESS;
        if(isSingleChannelu8(src))
            status = fcvWarpPerspectiveu8_v4(src,
                                             srcWidth,
                                             srcHeight,
                                             srcStride,
                                             dst,
                                             dstWidth,
                                             dstHeight,
                                             dstStride,
                                             projection,
                                             FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                             FASTCV_BORDER_CONSTANT,
                                             0);
        else
            fcv3ChannelWarpPerspectiveu8_v2(src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride, projection);
        if(status != fcvStatus::FASTCV_SUCCESS) {
            if(logger) logger->error("FastCV operation failed with error code {}", status);
            return false;
        }
    }
#else
    (void)(src);
    (void)(dst);
    (void)(srcWidth);
    (void)(srcHeight);
    (void)(srcStride);
    (void)(dstWidth);
    (void)(dstHeight);
    (void)(dstStride);
    (void)(numChannels);
    (void)(bpp);
    (void)(matrix);
    (void)(background);
    (void)(srcImgSpecs);
    (void)(sourceMinX);
    (void)(sourceMinY);
    (void)(sourceMaxX);
    (void)(sourceMaxY);
    (void)(fastCvBorder);
#endif
}

dai::impl::FrameSpecs dai::impl::getSrcFrameSpecs(dai::ImgFrame::Specs srcSpecs) {
    FrameSpecs specs;
    specs.width = srcSpecs.width;
    specs.height = srcSpecs.height;
    specs.p1Offset = srcSpecs.p1Offset;
    switch(srcSpecs.type) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : specs.width;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.p1Stride * specs.height;
            specs.p3Offset = srcSpecs.p3Offset > 0 ? srcSpecs.p3Offset : 2 * specs.p1Stride * specs.height;
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = srcSpecs.stride >= 3 * specs.width ? srcSpecs.stride : 3 * specs.width;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.width * specs.height;
            specs.p3Offset = specs.p2Offset;
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : (specs.p2Offset - specs.p1Offset) / specs.height;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.width * specs.height;
            specs.p3Offset = srcSpecs.p3Offset > srcSpecs.p2Offset ? srcSpecs.p3Offset : specs.p2Offset + (specs.width * specs.height) / 4;
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : (specs.p2Offset - specs.p1Offset) / specs.height;
            specs.p2Stride = (specs.p3Offset - specs.p2Offset) / (specs.height / 2);
            specs.p3Stride = specs.p2Stride;
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : specs.width;
            break;
        case ImgFrame::Type::RAW16:
            specs.p1Stride = srcSpecs.stride >= specs.width * 2 ? srcSpecs.stride : specs.width * 2;
            break;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            throw std::runtime_error("Frame type " + std::to_string((int)srcSpecs.type) + " not supported");
            break;
    }
    return specs;
}

dai::impl::FrameSpecs dai::impl::getDstFrameSpecs(size_t width, size_t height, dai::ImgFrame::Type type) {
    FrameSpecs specs;
    specs.width = width;
    specs.height = height;
    specs.p1Offset = 0;
    switch(type) {
        case dai::ImgFrame::Type::RGB888p:  // Do not do striding for RGB/BGRi/p
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = specs.width;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + specs.p1Stride * specs.height;
            specs.p3Offset = specs.p2Offset + specs.p1Stride * specs.height;
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = specs.width * 3;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p1Stride = ALIGN_UP(specs.width, DEPTHAI_STRIDE_ALIGNMENT);
            specs.p2Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * ALIGN_UP(specs.height, DEPTHAI_HEIGHT_ALIGNMENT), DEPTHAI_PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset;
            specs.p3Stride = 0;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p1Stride = ALIGN_UP(specs.width, DEPTHAI_STRIDE_ALIGNMENT);
            specs.p2Stride = ALIGN_UP(specs.width / 2, DEPTHAI_STRIDE_ALIGNMENT);
            specs.p3Stride = ALIGN_UP(specs.width / 2, DEPTHAI_STRIDE_ALIGNMENT);
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * ALIGN_UP(specs.height, DEPTHAI_HEIGHT_ALIGNMENT), DEPTHAI_PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p2Stride * ALIGN_UP(specs.height / 2, DEPTHAI_HEIGHT_ALIGNMENT / 2), DEPTHAI_PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = ALIGN_UP(specs.width, DEPTHAI_STRIDE_ALIGNMENT);
            break;
        case ImgFrame::Type::RAW16:  // Do not do alignment for RAW16
            specs.p1Stride = specs.width * 2;
            break;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            throw std::runtime_error("Frame type not supported");
            break;
    }
    return specs;
}

dai::impl::FrameSpecs dai::impl::getCcDstFrameSpecs(FrameSpecs srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to) {
    if(from == to)
        return srcSpecs;
    else
        return getDstFrameSpecs(srcSpecs.width, srcSpecs.height, to);
}

bool dai::impl::isTypeSupported(dai::ImgFrame::Type type) {
    using ImgType = dai::ImgFrame::Type;
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i;
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_OPENCV
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RAW16 || type == ImgType::RGB888i || type == ImgType::BGR888i
           || type == ImgType::RGB888p || type == ImgType::BGR888p || type == ImgType::YUV420p || type == ImgType::NV12;
#else
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i || type == ImgType::RGB888p
           || type == ImgType::BGR888p || type == ImgType::YUV420p || type == ImgType::NV12;
#endif
}

bool dai::impl::getFrameTypeInfo(dai::ImgFrame::Type outFrameType, int& outNumPlanes, float& outBpp) {
    // Set output Bpp and planes by PixelFormat and interleaved options
    outNumPlanes = 3;

    switch(outFrameType) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            outBpp = 1;
            outNumPlanes = 3;
            break;

        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            outBpp = 3;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::YUV400p:
        case dai::ImgFrame::Type::GRAY8:
            outBpp = 1;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::RAW16:
            outBpp = 2;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::YUV420p:
        case dai::ImgFrame::Type::NV12:
            outBpp = 1.5;
            outNumPlanes = 1;
            break;

        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            return false;
            break;
    }

    return true;
}

std::tuple<float, float, float, float> dai::impl::getOuterRect(const std::vector<std::array<float, 2>> points) {
    float minx = points[0][0];
    float maxx = points[0][0];
    float miny = points[0][1];
    float maxy = points[0][1];

    for(auto i = 0U; i < points.size(); ++i) {
        minx = std::min(points[i][0], minx);
        maxx = std::max(points[i][0], maxx);
        miny = std::min(points[i][1], miny);
        maxy = std::max(points[i][1], maxy);
    }
    return {minx, maxx, miny, maxy};
}

std::vector<std::array<float, 2>> dai::impl::getHull(const std::vector<std::array<float, 2>> points) {
    std::vector<std::array<float, 2>> remaining(points.rbegin(), points.rend() - 1);
    std::vector<std::array<float, 2>> hull{points.front()};
    hull.reserve(points.size());
    while(remaining.size() > 0) {
        auto pt = remaining.back();
        remaining.pop_back();
        while(hull.size() >= 2) {
            auto last1 = hull.size() - 1;
            auto last2 = hull.size() - 2;
            std::array<float, 2> v1 = {hull[last1][0] - hull[last2][0], hull[last1][1] - hull[last2][1]};
            std::array<float, 2> v2 = {pt[0] - hull[last1][0], pt[1] - hull[last1][1]};
            std::array<float, 2> v3 = {hull[0][0] - pt[0], hull[0][1] - pt[1]};
            auto cross1 = v1[0] * v2[1] - v1[1] * v2[0];
            auto cross2 = v2[0] * v3[1] - v2[1] * v3[0];
            if(cross1 < 0 || cross2 < 0) {
                remaining.push_back(hull.back());
                hull.pop_back();
            } else if(cross1 == 0 || cross2 == 0) {
                throw std::runtime_error("Colinear points");
            } else {
                break;
            }
        }
        hull.push_back(pt);
    }
    return hull;
}

std::array<std::array<float, 2>, 2> dai::impl::getInverse(const std::array<std::array<float, 2>, 2> mat) {
    auto det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    if(det == 0) {
        throw std::runtime_error("Determinant is zero");
    }
    return {{{mat[1][1] / det, -mat[0][1] / det}, {-mat[1][0] / det, mat[0][0] / det}}};
}

std::array<std::array<float, 3>, 3> dai::impl::getInverse(const std::array<std::array<float, 3>, 3>& matrix) {
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

dai::RotatedRect dai::impl::getOuterRotatedRect(const std::vector<std::array<float, 2>>& points) {
    return utility::getOuterRotatedRect(points);
}

std::array<std::array<float, 3>, 3> dai::impl::getResizeMat(Resize o, float width, float height, uint32_t outputWidth, uint32_t outputHeight) {
    if(o.mode == Resize::VALUE) {
        if(o.width > 0 && o.height <= 0) {
            o.height = o.normalized ? o.width : height * (o.width / width);
        } else if(o.width <= 0 && o.height > 0) {
            o.width = o.normalized ? o.height : width * (o.height / height);
        }
    } else if(o.mode == Resize::FIT || o.mode == Resize::FILL) {
        if(outputWidth > 0 && outputHeight > 0) {
            float ratio = width / height;
            if((float)outputWidth / (float)outputHeight > ratio) {
                if(o.mode == Resize::FIT) {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                } else {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                }
            } else {
                if(o.mode == Resize::FIT) {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                } else {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                }
            }
        } else {
            throw std::runtime_error("Neither output size nor resize dimensions are set");
        }
    }
    if(!o.normalized) {
        o.width /= width;
        o.height /= height;
    }
    return {{{o.width, 0, 0}, {0, o.height, 0}, {0, 0, 1}}};
}

void dai::impl::getOutputSizeFromCorners(const std::array<std::array<float, 2>, 4>& corners,
                                         const bool center,
                                         const std::array<std::array<float, 3>, 3> transformInv,
                                         const uint32_t srcWidth,
                                         const uint32_t srcHeight,
                                         uint32_t& outputWidth,
                                         uint32_t& outputHeight) {
    auto [dstMinx, dstMaxx, dstMiny, dstMaxy] = dai::impl::getOuterRect(std::vector(corners.begin(), corners.end()));

    float innerMinx = ceilf(dstMinx);
    float innerMiny = ceilf(dstMiny);
    float innerMaxx = floorf(dstMaxx);
    float innerMaxy = floorf(dstMaxy);
    float outerMinx = roundf(dstMinx);
    float outerMiny = roundf(dstMiny);
    float outerMaxx = roundf(dstMaxx);
    float outerMaxy = roundf(dstMaxy);

    std::array<std::array<float, 2>, 4> innerCorners = {{{innerMinx, innerMiny}, {innerMaxx, innerMiny}, {innerMaxx, innerMaxy}, {innerMinx, innerMaxy}}};
    std::array<std::array<float, 2>, 4> outerCorners = {{{outerMinx, outerMiny}, {outerMaxx, outerMiny}, {outerMaxx, outerMaxy}, {outerMinx, outerMaxy}}};
    std::array<std::array<float, 2>, 4> srcInnerCorners = {{matvecmul(transformInv, innerCorners[0]),
                                                            matvecmul(transformInv, innerCorners[1]),
                                                            matvecmul(transformInv, innerCorners[2]),
                                                            matvecmul(transformInv, innerCorners[3])}};
    std::array<std::array<float, 2>, 4> srcOuterCorners = {{matvecmul(transformInv, outerCorners[0]),
                                                            matvecmul(transformInv, outerCorners[1]),
                                                            matvecmul(transformInv, outerCorners[2]),
                                                            matvecmul(transformInv, outerCorners[3])}};

    auto [srcInnerMinx, srcInnerMaxx, srcInnerMiny, srcInnerMaxy] = dai::impl::getOuterRect(std::vector(srcInnerCorners.begin(), srcInnerCorners.end()));
    auto [srcOuterMinx, srcOuterMaxx, srcOuterMiny, srcOuterMaxy] = dai::impl::getOuterRect(std::vector(srcOuterCorners.begin(), srcOuterCorners.end()));

    // If outer bb is outside of the image, but inner is inside, use inner bb, otherwise use outer bb
    float rminx = srcOuterMinx < 0 && srcInnerMinx >= 0 ? innerMinx : outerMinx;
    float rmaxx = srcOuterMaxx >= srcWidth && srcInnerMaxx < srcWidth ? innerMaxx : outerMaxx;
    float rminy = srcOuterMiny < 0 && srcInnerMiny >= 0 ? innerMiny : outerMiny;
    float rmaxy = srcOuterMaxy >= srcHeight && srcInnerMaxy < srcHeight ? innerMaxy : outerMaxy;
    if(!center) {
        if(outputWidth == 0) outputWidth = rmaxx;
        if(outputHeight == 0) outputHeight = rmaxy;
    } else {
        if(outputWidth == 0) outputWidth = rmaxx - rminx;
        if(outputHeight == 0) outputHeight = rmaxy - rminy;
    }
}

void dai::impl::getTransformImpl(const ManipOp& op,
                                 std::array<std::array<float, 3>, 3>& transform,
                                 std::array<std::array<float, 2>, 4>& imageCorners,
                                 std::vector<std::array<std::array<float, 2>, 4>>& srcCorners,
                                 uint32_t& outputWidth,
                                 uint32_t& outputHeight) {
    std::array<std::array<float, 3>, 3> mat = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    float centerX = (imageCorners[0][0] + imageCorners[1][0] + imageCorners[2][0] + imageCorners[3][0]) / 4;
    float centerY = (imageCorners[0][1] + imageCorners[1][1] + imageCorners[2][1] + imageCorners[3][1]) / 4;

    const auto [_minx, _maxx, _miny, _maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
    float minx = _minx;
    float maxx = _maxx;
    float miny = _miny;
    float maxy = _maxy;
    float width = maxx - minx;
    float height = maxy - miny;

    std::visit(
        overloaded{[](auto _) {},
                   [&](Translate o) {
                       if(o.normalized) {
                           o.offsetX *= width;
                           o.offsetY *= height;
                       }
                       mat = {{{1, 0, o.offsetX}, {0, 1, o.offsetY}, {0, 0, 1}}};
                   },
                   [&](Rotate o) {
                       float cos = std::cos(o.angle);
                       float sin = std::sin(o.angle);
                       if(o.normalized) {
                           o.offsetX *= width;
                           o.offsetY *= height;
                       }
                       float moveX = centerX + o.offsetX;
                       float moveY = centerY + o.offsetY;
                       if(o.center) {
                           mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                       }
                       mat = matmul({{{cos, -sin, 0}, {sin, cos, 0}, {0, 0, 1}}}, mat);
                       if(o.center) {
                           mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                       }
                   },
                   [&](Resize o) { mat = getResizeMat(o, width, height, outputWidth, outputHeight); },
                   [&](Flip o) {
                       float moveX = centerX;
                       float moveY = centerY;
                       switch(o.direction) {
                           case Flip::HORIZONTAL: {
                               if(o.center) {
                                   mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                               }
                               mat = matmul({{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, mat);
                               if(o.center) {
                                   mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                               }
                               break;
                           }
                           case Flip::VERTICAL: {
                               if(o.center) {
                                   mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                               }
                               mat = matmul({{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}, mat);
                               if(o.center) {
                                   mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                               }
                               break;
                           }
                       }
                   },
                   [&](FourPoints o) {
                       if(o.normalized) {
                           for(auto i = 0; i < 4; ++i) {
                               o.src[i].x *= width;
                               o.src[i].y *= height;
                               o.dst[i].x *= width;
                               o.dst[i].y *= height;
                           }
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
                           std::array<float, 9> coeff = {};
                           std::array<float, 8> srcData = {o.src[0].x, o.src[0].y, o.src[1].x, o.src[1].y, o.src[2].x, o.src[2].y, o.src[3].x, o.src[3].y};
                           std::array<float, 8> dstData = {o.dst[0].x, o.dst[0].y, o.dst[1].x, o.dst[1].y, o.dst[2].x, o.dst[2].y, o.dst[3].x, o.dst[3].y};
                           fcvGetPerspectiveTransformf32(srcData.data(), dstData.data(), coeff.data());
                           mat = {{{coeff[0], coeff[1], coeff[2]}, {coeff[3], coeff[4], coeff[5]}, {coeff[6], coeff[7], coeff[8]}}};
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
                           cv::Point2f srcPoints[4] = {cv::Point2f(o.src[0].x, o.src[0].y),
                                                       cv::Point2f(o.src[1].x, o.src[1].y),
                                                       cv::Point2f(o.src[2].x, o.src[2].y),
                                                       cv::Point2f(o.src[3].x, o.src[3].y)};
                           cv::Point2f dstPoints[4] = {cv::Point2f(o.dst[0].x, o.dst[0].y),
                                                       cv::Point2f(o.dst[1].x, o.dst[1].y),
                                                       cv::Point2f(o.dst[2].x, o.dst[2].y),
                                                       cv::Point2f(o.dst[3].x, o.dst[3].y)};
                           cv::Mat cvMat = cv::getPerspectiveTransform(srcPoints, dstPoints);
                           mat = {{{(float)cvMat.at<double>(0, 0), (float)cvMat.at<double>(0, 1), (float)cvMat.at<double>(0, 2)},
                                   {(float)cvMat.at<double>(1, 0), (float)cvMat.at<double>(1, 1), (float)cvMat.at<double>(1, 2)},
                                   {(float)cvMat.at<double>(2, 0), (float)cvMat.at<double>(2, 1), (float)cvMat.at<double>(2, 2)}}};
#else
                           throw std::runtime_error("FourPoints not supported without OpenCV or FastCV enabled");
#endif
                       }
                   },
                   [&](Affine o) { mat = {{{o.matrix[0], o.matrix[1], 0}, {o.matrix[2], o.matrix[3], 0}, {0, 0, 1}}}; },
                   [&](Perspective o) {
                       mat = {{{o.matrix[0], o.matrix[1], o.matrix[2]}, {o.matrix[3], o.matrix[4], o.matrix[5]}, {o.matrix[6], o.matrix[7], o.matrix[8]}}};
                   },
                   [&](Crop o) {
                       if(o.normalized) {
                           o.width *= width;
                           o.height *= height;
                       } else if((o.width > 0 && o.width < 1) || (o.height > 0 && o.height < 1)) {
                           throw std::runtime_error("Crop not marked as normalized, but values seem to be normalized (height or width is less than 1)");
                       }
                       if(o.width > 0 && o.height <= 0)
                           o.height = roundf(height * ((float)o.width / width));
                       else if(o.height > 0 && o.width <= 0)
                           o.width = roundf(width * (o.height / height));
                       else if(o.height <= 0 && o.width <= 0) {
                           o.width = roundf(maxx);
                           o.height = roundf(maxy);
                       }

                       outputWidth = o.width;
                       outputHeight = o.height;

                       if(o.center) {
                           std::array<std::array<float, 3>, 3> _mat = {
                               {{1, 0, -minx + (outputWidth - (maxx - minx)) / 2}, {0, 1, -miny + (outputHeight - (maxy - miny)) / 2}, {0, 0, 1}}};
                           transform = matmul(_mat, transform);
                       }

                       imageCorners = {{{0, 0}, {(float)outputWidth, 0}, {(float)outputWidth, (float)outputHeight}, {0, (float)outputHeight}}};
                       auto transformInv = getInverse(transform);
                       srcCorners.push_back({matvecmul(transformInv, imageCorners[0]),
                                             matvecmul(transformInv, imageCorners[1]),
                                             matvecmul(transformInv, imageCorners[2]),
                                             matvecmul(transformInv, imageCorners[3])});
                   }},
        op.op);
    auto outerRectPoints =
        getOuterRotatedRect(
            {matvecmul(mat, imageCorners[0]), matvecmul(mat, imageCorners[1]), matvecmul(mat, imageCorners[2]), matvecmul(mat, imageCorners[3])})
            .getPoints();
    for(auto i = 0; i < 4; ++i) {
        imageCorners[i] = {outerRectPoints[i].x, outerRectPoints[i].y};
    }
    transform = matmul(mat, transform);
}

size_t dai::impl::getFrameSize(const ImgFrame::Type type, const FrameSpecs& specs) {
    switch(type) {
        case ImgFrame::Type::YUV420p:
            return specs.p3Offset + specs.p3Stride * specs.height / 2;
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            return specs.p3Offset + specs.p3Stride * specs.height;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            return specs.p1Stride * specs.height;
        case ImgFrame::Type::NV12:
            return specs.p2Offset + specs.p2Stride * specs.height / 2;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW16:
            return specs.p1Stride * specs.height;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }
    return 0;
}
void dai::impl::printSpecs(spdlog::async_logger& logger, FrameSpecs specs) {
    logger.debug("Width: {}, Height: {}, P1Offset: {}, P1Stride: {}, P2Offset: {}, P2Stride: {}, P3Offset: {}, P3Stride: {}",
                 specs.width,
                 specs.height,
                 specs.p1Offset,
                 specs.p1Stride,
                 specs.p2Offset,
                 specs.p2Stride,
                 specs.p3Offset,
                 specs.p3Stride);
}

size_t dai::impl::getAlignedOutputFrameSize(ImgFrame::Type type, size_t width, size_t height) {
    auto alignWidth = [](size_t _width) -> size_t { return ALIGN_UP(_width, DEPTHAI_STRIDE_ALIGNMENT); };
    auto alignHeight = [](size_t _height, int fx = 1) -> size_t { return ALIGN_UP(_height, DEPTHAI_HEIGHT_ALIGNMENT / fx); };
    auto alignSize = [](size_t _size) -> size_t { return ALIGN_UP(_size, DEPTHAI_PLANE_ALIGNMENT); };
    switch(type) {
        case ImgFrame::Type::YUV420p:
            return alignSize(alignWidth(width) * alignHeight(height)) + 2 * alignSize(alignWidth(width / 2) * alignHeight(height / 2, 2));
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            return 3 * alignSize(alignWidth(width) * alignHeight(height));
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            return alignSize(alignWidth(3 * width) * alignHeight(height));
        case ImgFrame::Type::NV12:
            return alignSize(alignWidth(width) * alignHeight(height)) + alignSize(alignWidth(width) * alignHeight(height / 2, 2));
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            return alignSize(alignWidth(width) * alignHeight(height));
        case ImgFrame::Type::RAW16:
            return alignSize(alignWidth(width) * alignHeight(height) * 2);
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }
    return 0;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
bool dai::impl::UndistortOpenCvImpl::validMatrix(std::array<float, 9> matrix) const {
    return !floatEq(matrix[0], 0) && floatEq(matrix[1], 0) && floatEq(matrix[3], 0) && !floatEq(matrix[4], 0) && floatEq(matrix[6], 0) && floatEq(matrix[7], 0)
           && floatEq(matrix[8], 1);
}
void dai::impl::UndistortOpenCvImpl::initMaps(std::array<float, 9> cameraMatrix,
                                              std::array<float, 9> newCameraMatrix,
                                              std::vector<float> distCoeffs,
                                              dai::ImgFrame::Type type,
                                              uint32_t srcWidth,
                                              uint32_t srcHeight,
                                              uint32_t dstWidth,
                                              uint32_t dstHeight) {
    this->cameraMatrix = std::move(cameraMatrix);
    this->newCameraMatrix = std::move(newCameraMatrix);
    this->distCoeffs = std::move(distCoeffs);
    this->type = type;
    this->srcWidth = srcWidth;
    this->srcHeight = srcHeight;
    this->dstWidth = dstWidth;
    this->dstHeight = dstHeight;
    undistortMap1 = cv::Mat();
    undistortMap2 = cv::Mat();
    undistortMap1Half = cv::Mat();
    undistortMap2Half = cv::Mat();

    cv::Mat cvCameraMatrix(3, 3, CV_32F, this->cameraMatrix.data());
    cv::Mat cvNewCameraMatrix(3, 3, CV_32F, this->newCameraMatrix.data());
    cv::initUndistortRectifyMap(
        cvCameraMatrix, this->distCoeffs, cv::Mat(), cvNewCameraMatrix, cv::Size(dstWidth, dstHeight), CV_16SC2, undistortMap1, undistortMap2);
    if(type == dai::ImgFrame::Type::NV12 || type == dai::ImgFrame::Type::YUV420p) {
        cv::Mat cvCameraMatrixHalf = cvCameraMatrix.clone();
        cv::Mat cvNewCameraMatrixHalf = cvNewCameraMatrix.clone();
        cvCameraMatrixHalf.at<float>(0, 0) /= 2;
        cvCameraMatrixHalf.at<float>(1, 1) /= 2;
        cvCameraMatrixHalf.at<float>(0, 2) /= 2;
        cvCameraMatrixHalf.at<float>(1, 2) /= 2;
        cvNewCameraMatrixHalf.at<float>(0, 0) /= 2;
        cvNewCameraMatrixHalf.at<float>(1, 1) /= 2;
        cvNewCameraMatrixHalf.at<float>(0, 2) /= 2;
        cvNewCameraMatrixHalf.at<float>(1, 2) /= 2;
        cv::initUndistortRectifyMap(cvCameraMatrixHalf,
                                    this->distCoeffs,
                                    cv::Mat(),
                                    cvNewCameraMatrixHalf,
                                    cv::Size(dstWidth / 2, dstHeight / 2),
                                    CV_16SC2,
                                    undistortMap1Half,
                                    undistortMap2Half);
    }
}
dai::impl::UndistortOpenCvImpl::BuildStatus dai::impl::UndistortOpenCvImpl::build(std::array<float, 9> cameraMatrix,
                                                                                  std::array<float, 9> newCameraMatrix,
                                                                                  std::vector<float> distCoeffs,
                                                                                  dai::ImgFrame::Type type,
                                                                                  uint32_t srcWidth,
                                                                                  uint32_t srcHeight,
                                                                                  uint32_t dstWidth,
                                                                                  uint32_t dstHeight) {
    if(!distCoeffs.empty()) {
        if(!validMatrix(cameraMatrix)) {
            logger->error("Invalid camera matrix provided for undistortion, will not be applied ({},{},{},{},{},{},{},{},{})",
                          cameraMatrix[0],
                          cameraMatrix[1],
                          cameraMatrix[2],
                          cameraMatrix[3],
                          cameraMatrix[4],
                          cameraMatrix[5],
                          cameraMatrix[6],
                          cameraMatrix[7],
                          cameraMatrix[8]);
            return BuildStatus::ERROR;
        }
        if(!validMatrix(newCameraMatrix)) {
            if(type != this->type || srcWidth != this->srcWidth || srcHeight != this->srcHeight || distCoeffs != this->distCoeffs
               || cameraMatrix != this->cameraMatrix) {
                initMaps(cameraMatrix, cameraMatrix, std::move(distCoeffs), type, srcWidth, srcHeight, srcWidth, srcHeight);
                return BuildStatus::TWO_SHOT;
            }
            return BuildStatus::NOT_BUILT;
        } else {
            if(type != this->type || srcWidth != this->srcWidth || srcHeight != this->srcHeight || dstWidth != this->dstWidth || dstHeight != this->dstHeight
               || distCoeffs != this->distCoeffs || cameraMatrix != this->cameraMatrix || newCameraMatrix != this->newCameraMatrix) {
                initMaps(std::move(cameraMatrix), std::move(newCameraMatrix), std::move(distCoeffs), type, srcWidth, srcHeight, dstWidth, dstHeight);
                return BuildStatus::ONE_SHOT;
            }
            return BuildStatus::NOT_BUILT;
        }
    }
    return BuildStatus::NOT_USED;
}
void dai::impl::UndistortOpenCvImpl::undistort(cv::Mat& src, cv::Mat& dst) {
    if(dst.size().width == (int)dstWidth && dst.size().height == (int)dstHeight) {
        cv::remap(src, dst, undistortMap1, undistortMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    } else if(dst.size().width == (int)dstWidth / 2 && dst.size().height == (int)dstHeight / 2) {
        if(undistortMap1Half.empty() || undistortMap2Half.empty()) {
            throw std::runtime_error(
                "UndistortImpl: Undistort maps for this type are not initialized");  // This should not happen, the maps are initialized for NV12 and YUV420p
        }
        cv::remap(src, dst, undistortMap1Half, undistortMap2Half, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127));
    } else {
        throw std::runtime_error(fmt::format("UndistortImpl: Output size does not match the expected size (got {}x{}, expected {}x{} or {}x{})",
                                             dst.size().width,
                                             dst.size().height,
                                             dstWidth,
                                             dstHeight,
                                             dstWidth / 2,
                                             dstHeight / 2));
    }
}
#endif

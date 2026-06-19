#include "depthai/utility/ImageManipImpl.hpp"

#include <algorithm>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/utility/OCVPorts.hpp"
#include "depthai/utility/matrixOps.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
#endif

#if defined(WIN32) || defined(_WIN32)
    #define _RESTRICT
#else
    #define _RESTRICT __restrict__
#endif

constexpr size_t MAX_AUTO_WIDTH = 4000;

constexpr size_t MAX_AUTO_HEIGHT = 3000;

// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

static inline int clampi(int val, int minv, int maxv) {
    // return val < minv ? minv : (val > maxv ? maxv : val);
    return std::clamp(val, minv, maxv);
}
inline bool floatEq(float a, float b) {
    return fabsf(a - b) <= 1e-6f;
}
inline bool isSingleChannel(const dai::ImgFrame::Type type) {
    return type == dai::ImgFrame::Type::GRAY8 || type == dai::ImgFrame::Type::RAW8 || type == dai::ImgFrame::Type::RAW16 || type == dai::ImgFrame::Type::GRAYF16
           || type == dai::ImgFrame::Type::RAW32;
}

void transformOpenCV(const uint8_t* src,
                     uint8_t* dst,
                     const size_t srcWidth,
                     const size_t srcHeight,
                     const size_t srcStride,
                     const size_t dstWidth,
                     const size_t dstHeight,
                     const size_t dstStride,
                     const uint16_t numChannels,
                     const uint16_t bpp,
                     const std::array<std::array<float, 3>, 3>& matrix,
                     const std::vector<uint32_t>& background,
                     const dai::impl::FrameSpecs& srcImgSpecs,
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
bool transformFastCV(const uint8_t* src,
                     uint8_t* dst,
                     const size_t srcWidth,
                     const size_t srcHeight,
                     const size_t srcStride,
                     const size_t dstWidth,
                     const size_t dstHeight,
                     const size_t dstStride,
                     const uint16_t numChannels,
                     const uint16_t bpp,
                     const std::array<std::array<float, 3>, 3>& matrix,
                     const std::vector<uint32_t>& background,
                     const dai::impl::FrameSpecs& srcImgSpecs,
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
    int ssF = srcImgSpecs.width / srcWidth;
    assert(ssF == (int)(srcImgSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));  // Sanity check
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
    return false;
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

dai::impl::FrameSpecs dai::impl::getCcDstFrameSpecs(const FrameSpecs& srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to) {
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
    using ImgFrame = dai::ImgFrame;

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

std::tuple<float, float, float, float> dai::impl::getOuterRect(const std::vector<std::array<float, 2>>& points) {
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
                                         const std::array<std::array<float, 3>, 3>& transformInv,
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

    if(outputWidth > 0 && outputHeight > 0) return;

    if(!center) {
        if(outputWidth == 0) outputWidth = std::min((size_t)rmaxx, MAX_AUTO_WIDTH);
        if(outputHeight == 0) outputHeight = std::min((size_t)rmaxy, MAX_AUTO_HEIGHT);
    } else {
        if(outputWidth == 0) outputWidth = std::min((size_t)std::max(rmaxx - rminx, 0.f), MAX_AUTO_WIDTH);
        if(outputHeight == 0) outputHeight = std::min((size_t)std::max(rmaxy - rminy, 0.f), MAX_AUTO_HEIGHT);
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
                   [&](const Resize& o) { mat = getResizeMat(o, width, height, outputWidth, outputHeight); },
                   [&](const Flip& o) {
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
                       mat = matrix::getHomographyMatrix(o.src, o.dst);
#endif
                   },
                   [&](const Affine& o) { mat = {{{o.matrix[0], o.matrix[1], 0}, {o.matrix[2], o.matrix[3], 0}, {0, 0, 1}}}; },
                   [&](const Perspective& o) {
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
                       auto transformInv = matrix::getMatrixInverse(transform);
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
bool dai::impl::UndistortOpenCvImpl::validMatrix(const std::array<float, 9>& matrix) const {
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

namespace dai::impl {

//--------------------------------------------------
//------------------ Color Conversion --------------
//--------------------------------------------------

static inline void YUVfromRGB(float& Y, float& U, float& V, const float R, const float G, const float B) {
    Y = 0.257f * R + 0.504f * G + 0.098f * B + 16;
    U = -0.148f * R - 0.291f * G + 0.439f * B + 128;
    V = 0.439f * R - 0.368f * G - 0.071f * B + 128;
}
static inline void RGBfromYUV(float& R, float& G, float& B, float Y, float U, float V) {
    Y -= 16;
    U -= 128;
    V -= 128;
    R = 1.164f * Y + 1.596f * V;
    G = 1.164f * Y - 0.392f * U - 0.813f * V;
    B = 1.164f * Y + 2.017f * U;
}

bool colorConvertToRGB888p(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888p;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame->getOffsetData().data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame->getOffsetData().data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame->getOffsetData().data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame->getOffsetData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getOffsetData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getOffsetData()[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame->getOffsetData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getOffsetData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getOffsetData()[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->getOffsetData().data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(auxBGR, channels);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->getOffsetData().data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    const uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + (uint32_t)(j / 2)];
                    float V = src[lineStartV + (uint32_t)(j / 2)];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                    outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                }
            }
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool colorConvertToBGR888p(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888p;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame->getOffsetData().data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame->getOffsetData().data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame->getOffsetData().data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
            break;
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame->getOffsetData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getOffsetData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getOffsetData()[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame->getOffsetData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getOffsetData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getOffsetData()[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->getOffsetData().data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(auxBGR, channels);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->getOffsetData().data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->getOffsetData().data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    const uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + (uint32_t)(j / 2)];
                    float V = src[lineStartV + (uint32_t)(j / 2)];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                    outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                }
            }
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool colorConvertToRGB888i(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888i;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame->getOffsetData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getOffsetData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getOffsetData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame->getOffsetData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getOffsetData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getOffsetData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset,
                                     srcSpecs.width,
                                     srcSpecs.height,
                                     srcSpecs.p1Stride,
                                     outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                     dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame->getOffsetData()[dstPos + 0] = src[srcPos + 2];
                    outputFrame->getOffsetData()[dstPos + 1] = src[srcPos + 1];
                    outputFrame->getOffsetData()[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, img, cv::COLOR_RGB2BGR);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                             dstSpecs.p1Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + (uint32_t)(j / 2)];
                    float V = src[lineStartV + (uint32_t)(j / 2)];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame->getOffsetData()[pos + 0] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                    outputFrame->getOffsetData()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame->getOffsetData()[pos + 2] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                }
            }
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool colorConvertToBGR888i(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888i;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
#endif

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame->getOffsetData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getOffsetData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getOffsetData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame->getOffsetData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getOffsetData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getOffsetData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset,
                                     srcSpecs.width,
                                     srcSpecs.height,
                                     srcSpecs.p1Stride,
                                     outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                     dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame->getOffsetData()[dstPos + 0] = src[srcPos + 2];
                    outputFrame->getOffsetData()[dstPos + 1] = src[srcPos + 1];
                    outputFrame->getOffsetData()[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
            break;
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->getOffsetData().data(),
                                                   auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->getOffsetData().data(),
                                     srcSpecs.width,
                                     srcSpecs.height,
                                     auxStride,
                                     outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                     dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColorTwoPlane(frameY, frameUV, img, cv::COLOR_YUV2BGR_NV12);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->getOffsetData().data(),
                                             auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->getOffsetData().data(),
                                     srcSpecs.width,
                                     srcSpecs.height,
                                     auxStride,
                                     outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                     dstSpecs.p1Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + (uint32_t)(j / 2)];
                    float V = src[lineStartV + (uint32_t)(j / 2)];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame->getOffsetData()[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    outputFrame->getOffsetData()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame->getOffsetData()[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                }
            }
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool colorConvertToNV12(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::NV12;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
#endif

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->getOffsetData().data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->getOffsetData().data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartB = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartR = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->getOffsetData().data(), auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->getOffsetData().data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[pos + 0];
                    float G = src[pos + 1];
                    float B = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(src + srcSpecs.p1Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                                   outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float B = src[pos + 0];
                    float G = src[pos + 1];
                    float R = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
            break;
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                src + srcSpecs.p3Offset,
                                srcSpecs.p3Stride,
                                FASTCV_CHANNEL_0,
                                FASTCV_IYUV,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelCombine2Planesu8(src + srcSpecs.p2Offset,
                                       srcSpecs.width / 2,
                                       srcSpecs.height / 2,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                       dstSpecs.p2Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat frameUV(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC2, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            cv::merge(channels, frameUV);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            memset(outputFrame->getOffsetData().data() + dstSpecs.p2Offset, 128, dstSpecs.p2Stride * dstSpecs.height / 2);
            done = true;
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
        case ImgFrame::Type::RAW16:
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

    return done;
}

bool colorConvertToYUV420p(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::YUV420p;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
#endif

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->getOffsetData().data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->getOffsetData().data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float B = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float R = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->getOffsetData().data(), auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->getOffsetData().data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float R = src[pos + 0];
                    float G = src[pos + 1];
                    float B = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToYCbCr420Planaru8(src + srcSpecs.p1Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                             outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float B = src[pos + 0];
                    float G = src[pos + 1];
                    float R = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame->getOffsetData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getOffsetData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getOffsetData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_Y,
                                FASTCV_NV12,
                                outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_U,
                                FASTCV_NV12,
                                outputFrame->getOffsetData().data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_V,
                                FASTCV_NV12,
                                outputFrame->getOffsetData().data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(frameUV, channels);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
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
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool colorConvertToGRAY8(const ColorChangeArgs& args) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::GRAY8;
    const std::shared_ptr<OffsetMemory>& inputFrame = args.inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame = args.outputFrame;
    FrameSpecs srcSpecs = args.srcSpecs;
    FrameSpecs dstSpecs = args.dstSpecs;
    ImgFrame::Type from = args.from;
    std::shared_ptr<OffsetMemory> ccAuxFrame = args.auxFrame;

    auto src = inputFrame->getOffsetData().data();
    auto inputSize = inputFrame->getOffsetSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->getOffsetData().data(),
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   auxStride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::merge(channels, auxRGB);
            // Convert to grayscale
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxRGB, gray, cv::COLOR_RGB2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->getOffsetData().data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->getOffsetData().data(),
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   auxStride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::merge(channels, auxRGB);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxRGB, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToGrayu8(src + srcSpecs.p1Offset,
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   srcSpecs.p1Stride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(frameRGB, gray, cv::COLOR_RGB2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->getOffsetData().data(), auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->getOffsetData().data(),
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   auxStride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(frameBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->getOffsetData().data(),
                                                   auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->getOffsetData().data(),
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   auxStride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->getOffsetData().data(),
                                             auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->getOffsetData().data(),
                                   srcSpecs.width,
                                   srcSpecs.height,
                                   auxStride,
                                   outputFrame->getOffsetData().data() + dstSpecs.p1Offset,
                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = srcSpecs.p1Offset + i * auxStride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + (uint32_t)(j / 2)];
                    float V = src[lineStartV + (uint32_t)(j / 2)];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    ccAuxFrame->getOffsetData().data()[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    ccAuxFrame->getOffsetData().data()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    ccAuxFrame->getOffsetData().data()[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                }
            }
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->getOffsetData().data(), auxStride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->getOffsetData().data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame->getOffsetData().data());
            done = true;
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
        case ImgFrame::Type::RAW16:
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

    return done;
}

void ColorChangeH::build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo) {
    from = typeFrom;
    to = typeTo;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;
    size_t newAuxFrameSize = ALIGN_UP(srcSpecs.height, DEPTHAI_HEIGHT_ALIGNMENT) * ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
    if(!ccAuxFrame || ccAuxFrame->getOffsetSize() < newAuxFrameSize) ccAuxFrame = std::make_shared<_ImageManipMemory>(newAuxFrameSize);
}

void ColorChangeH::apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
    float bpp;
    int numPlanes;
    getFrameTypeInfo(to, numPlanes, bpp);

    bool done = false;
    auto start = std::chrono::steady_clock::now();
    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
            done = colorConvertToRGB888p({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::BGR888p:
            done = colorConvertToBGR888p({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::RGB888i:
            done = colorConvertToRGB888i({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::BGR888i:
            done = colorConvertToBGR888i({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::NV12:
            done = colorConvertToNV12({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::YUV420p:
            done = colorConvertToYUV420p({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::GRAY8:
        case dai::ImgFrame::Type::RAW8:
            done = colorConvertToGRAY8({src, dst, srcSpecs, dstSpecs, from, ccAuxFrame});
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
        case ImgFrame::Type::RAW16:
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
    auto diff = std::chrono::steady_clock::now() - start;
    if(logger) logger->trace("ImageManip | colorConvert took {}ns", std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count());

    if(!done) {
        if(logger) logger->error("Convert color from {} to {} not supported or failed.", (int)from, (int)to);
        std::copy(src->getOffsetData().data(),
                  src->getOffsetData().data() + (src->getOffsetSize() <= dst->getOffsetSize() ? src->getOffsetSize() : dst->getOffsetSize()),
                  dst->getOffsetData().data());
    }
}

//--------------------------------------------------
//----------------------- Warp ---------------------
//--------------------------------------------------

bool Warp::isIdentityWarp() const {
    return (matrix[0][0] == 1.0f && matrix[0][1] == 0.0f && matrix[0][2] == 0.0f && matrix[1][0] == 0.0f && matrix[1][1] == 1.0f && matrix[1][2] == 0.0f
            && matrix[2][0] == 0.0f && matrix[2][1] == 0.0f && matrix[2][2] == 1.0f)
           && (srcSpecs.width == dstSpecs.width && srcSpecs.height == dstSpecs.height);
}

Warp& Warp::setBackgroundColor(const uint32_t r, const uint32_t g, const uint32_t b) {
    background = ImageManipOpsBase<Container>::Background::COLOR;
    switch(type) {
        case ImgFrame::Type::YUV420p:
        case ImgFrame::Type::NV12: {
            float y, u, v;
            YUVfromRGB(y, u, v, r, g, b);
            backgroundColor[0] = std::round(y);
            backgroundColor[1] = std::round(u);
            backgroundColor[2] = std::round(v);
            break;
        }
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::RGB888i:
            backgroundColor[0] = r;
            backgroundColor[1] = g;
            backgroundColor[2] = b;
            break;
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::BGR888i:
            backgroundColor[0] = b;
            backgroundColor[1] = g;
            backgroundColor[2] = r;
            break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            // backgroundColor[0] = 0.299f * r + 0.587f * g + 0.114f * b;
            backgroundColor[0] = b;
            break;
        case ImgFrame::Type::RAW16:
            backgroundColor[0] = r;
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
            break;
    }
    return *this;
}

void WarpH::build(const FrameSpecs srcFrameSpecs,
                  const FrameSpecs dstFrameSpecs,
                  const ImgFrame::Type type,
                  const std::array<std::array<float, 3>, 3> matrix,
                  std::vector<std::array<std::array<float, 2>, 4>> srcCorners) {
    this->matrix = matrix;
    this->type = type;
    this->srcSpecs = srcFrameSpecs;
    this->dstSpecs = dstFrameSpecs;

    if(!fastCvBorder || fastCvBorder->size() < this->dstSpecs.height * 2)
        fastCvBorder = std::make_shared<_ImageManipBuffer<uint32_t>>(this->dstSpecs.height * 2);

    const uint32_t inWidth = srcFrameSpecs.width;
    const uint32_t inHeight = srcFrameSpecs.height;
    this->sourceMinX = 0;
    this->sourceMaxX = inWidth;
    this->sourceMinY = 0;
    this->sourceMaxY = inHeight;
    for(const auto& corners : srcCorners) {
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector<std::array<float, 2>>(corners.begin(), corners.end()));
        this->sourceMinX = std::max(this->sourceMinX, (size_t)std::floor(std::max(minx, 0.f)));
        this->sourceMinY = std::max(this->sourceMinY, (size_t)std::floor(std::max(miny, 0.f)));
        this->sourceMaxX = std::min(this->sourceMaxX, (size_t)std::ceil(maxx));
        this->sourceMaxY = std::min(this->sourceMaxY, (size_t)std::ceil(maxy));
    }
    if(this->sourceMinX >= this->sourceMaxX || this->sourceMinY >= this->sourceMaxY) throw std::runtime_error("Initial crop is outside the source image");
}

void WarpH::buildUndistort(bool enable,
                           const std::array<float, 9>& cameraMatrix,
                           const std::array<float, 9>& newCameraMatrix,
                           const std::vector<float>& distCoeffs,
                           const ImgFrame::Type type,
                           const uint32_t srcWidth,
                           const uint32_t srcHeight,
                           const uint32_t dstWidth,
                           const uint32_t dstHeight) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(enable) {
        if(!undistortImpl) undistortImpl = std::make_unique<UndistortOpenCvImpl>(this->logger);
        auto undistortStatus = undistortImpl->build(cameraMatrix, newCameraMatrix, distCoeffs, type, srcWidth, srcHeight, dstWidth, dstHeight);
        switch(undistortStatus) {
            case UndistortOpenCvImpl::BuildStatus::ONE_SHOT:
                this->enableUndistort = true;
                this->undistortOneShot = true;
                break;
            case UndistortOpenCvImpl::BuildStatus::TWO_SHOT:
                this->enableUndistort = true;
                this->undistortOneShot = false;
                break;
            case UndistortOpenCvImpl::BuildStatus::NOT_USED:
                this->enableUndistort = false;
                this->undistortOneShot = false;
                break;
            case UndistortOpenCvImpl::BuildStatus::NOT_BUILT:
                break;
            case UndistortOpenCvImpl::BuildStatus::ERROR:
                this->enableUndistort = false;
                break;
        }

        if(this->enableUndistort && !this->undistortOneShot) {
            auto frameSize = getAlignedOutputFrameSize(type, srcWidth, srcHeight);
            if(!auxFrame || auxFrame->getOffsetSize() < frameSize) {
                // When undistort is needed but cannot one shot - undistorted frame must be written to an aux buffer
                auxFrame = std::make_shared<_ImageManipMemory>(frameSize);
            }
        }
    } else {
        undistortImpl = nullptr;
        this->enableUndistort = false;
    }
#else
    (void)enable;
    (void)cameraMatrix;
    (void)newCameraMatrix;
    (void)distCoeffs;
    (void)type;
    (void)srcWidth;
    (void)srcHeight;
    (void)dstWidth;
    (void)dstHeight;
    throw std::runtime_error("Undistort requires OpenCV support");
#endif
}

void WarpH::transform(const std::shared_ptr<OffsetMemory>& src,
                      const std::shared_ptr<OffsetMemory>& dst,
                      const size_t srcWidth,
                      const size_t srcHeight,
                      const size_t srcStride,
                      const size_t dstWidth,
                      const size_t dstHeight,
                      const size_t dstStride,
                      const uint16_t numChannels,
                      const uint16_t bpp,
                      const std::array<std::array<float, 3>, 3>& matrix,
                      const std::vector<uint32_t>& background) {
#ifdef DEPTHAI_IMAGEMANIPV2_OPENCV
    transformOpenCV(src->getOffsetData().data(),
                    dst->getOffsetData().data(),
                    srcWidth,
                    srcHeight,
                    srcStride,
                    dstWidth,
                    dstHeight,
                    dstStride,
                    numChannels,
                    bpp,
                    matrix,
                    background,
                    this->srcSpecs,
                    this->sourceMinX,
                    this->sourceMinY,
                    this->sourceMaxX,
                    this->sourceMaxY);
#else
    throw std::runtime_error("OpenCV backend not available");
#endif

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && !defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    (void)src;
    (void)dst;
    (void)srcWidth;
    (void)srcHeight;
    (void)srcStride;
    (void)dstWidth;
    (void)dstHeight;
    (void)dstStride;
    (void)numChannels;
    (void)bpp;
    (void)matrix;
    (void)background;
#endif
}

void WarpH::apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
    auto undistortDst = this->isIdentityWarp() || this->undistortOneShot ? dst : auxFrame;
    auto undistortSpecs =
        this->isIdentityWarp() || this->undistortOneShot ? this->dstSpecs : getDstFrameSpecs(this->srcSpecs.width, this->srcSpecs.height, this->type);
    auto warpSrc = this->enableUndistort ? auxFrame : src;
    auto warpSrcSpecs = this->enableUndistort ? undistortSpecs : this->srcSpecs;
    // Apply transformation multiple times depending on the image format
    switch(this->type) {
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                cv::Mat srcCv(this->srcSpecs.height,
                              this->srcSpecs.width,
                              CV_8UC3,
                              src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                              this->srcSpecs.p1Stride);
                cv::Mat dstCv(undistortSpecs.height,
                              undistortSpecs.width,
                              CV_8UC3,
                              undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                              undistortSpecs.p1Stride);
                this->undistortImpl->undistort(srcCv, dstCv);
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          3,
                          1,
                          this->matrix,
                          {this->backgroundColor[0], this->backgroundColor[1], this->backgroundColor[2]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
            break;
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::RGB888p:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                                  this->srcSpecs.p1Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                                  undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p2Offset)->getOffsetData().data(),
                                  this->srcSpecs.p2Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p2Offset)->getOffsetData().data(),
                                  undistortSpecs.p2Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p3Offset)->getOffsetData().data(),
                                  this->srcSpecs.p3Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p3Offset)->getOffsetData().data(),
                                  undistortSpecs.p3Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[0]});
                transform(warpSrc->offset(warpSrcSpecs.p2Offset),
                          dst->offset(this->dstSpecs.p2Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p2Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p2Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[1]});
                transform(warpSrc->offset(warpSrcSpecs.p3Offset),
                          dst->offset(this->dstSpecs.p3Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p3Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p3Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[2]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
            break;
        case ImgFrame::Type::YUV420p:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                                  this->srcSpecs.p1Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                                  undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height / 2,
                                  this->srcSpecs.width / 2,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p2Offset)->getOffsetData().data(),
                                  this->srcSpecs.p2Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p2Offset)->getOffsetData().data(),
                                  undistortSpecs.p2Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height / 2,
                                  this->srcSpecs.width / 2,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p3Offset)->getOffsetData().data(),
                                  this->srcSpecs.p3Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p3Offset)->getOffsetData().data(),
                                  undistortSpecs.p3Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[0]});
                transform(warpSrc->offset(warpSrcSpecs.p2Offset),
                          dst->offset(this->dstSpecs.p2Offset),
                          warpSrcSpecs.width / 2,
                          warpSrcSpecs.height / 2,
                          warpSrcSpecs.p2Stride,
                          this->dstSpecs.width / 2,
                          this->dstSpecs.height / 2,
                          this->dstSpecs.p2Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[1]});
                transform(warpSrc->offset(warpSrcSpecs.p3Offset),
                          dst->offset(this->dstSpecs.p3Offset),
                          warpSrcSpecs.width / 2,
                          warpSrcSpecs.height / 2,
                          warpSrcSpecs.p3Stride,
                          this->dstSpecs.width / 2,
                          this->dstSpecs.height / 2,
                          this->dstSpecs.p3Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[2]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
            break;
        case ImgFrame::Type::NV12:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                                  this->srcSpecs.p1Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                                  undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height / 2,
                                  this->srcSpecs.width / 2,
                                  CV_8UC2,
                                  src->offset(this->srcSpecs.p2Offset)->getOffsetData().data(),
                                  this->srcSpecs.p2Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC2,
                                  undistortDst->offset(undistortSpecs.p2Offset)->getOffsetData().data(),
                                  undistortSpecs.p2Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[0]});
                transform(warpSrc->offset(warpSrcSpecs.p2Offset),
                          dst->offset(this->dstSpecs.p2Offset),
                          warpSrcSpecs.width / 2,
                          warpSrcSpecs.height / 2,
                          warpSrcSpecs.p2Stride,
                          this->dstSpecs.width / 2,
                          this->dstSpecs.height / 2,
                          this->dstSpecs.p2Stride,
                          2,
                          1,
                          this->matrix,
                          {this->backgroundColor[1], this->backgroundColor[2]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
            break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_8UC1,
                                  src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                                  this->srcSpecs.p1Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                                  undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          1,
                          1,
                          this->matrix,
                          {this->backgroundColor[0]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
            break;
        case ImgFrame::Type::RAW16:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            if(this->enableUndistort) {
                {
                    cv::Mat srcCv(this->srcSpecs.height,
                                  this->srcSpecs.width,
                                  CV_16UC1,
                                  src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                                  this->srcSpecs.p1Stride);
                    cv::Mat dstCv(undistortSpecs.height,
                                  undistortSpecs.width,
                                  CV_16UC1,
                                  undistortDst->offset(undistortSpecs.p1Offset)->getOffsetData().data(),
                                  undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
            }
    #endif
            if(!this->undistortOneShot && !this->isIdentityWarp()) {
                transform(warpSrc->offset(warpSrcSpecs.p1Offset),
                          dst->offset(this->dstSpecs.p1Offset),
                          warpSrcSpecs.width,
                          warpSrcSpecs.height,
                          warpSrcSpecs.p1Stride,
                          this->dstSpecs.width,
                          this->dstSpecs.height,
                          this->dstSpecs.p1Stride,
                          1,
                          2,
                          this->matrix,
                          {this->backgroundColor[0]});
            }
#else
            (void)src;
            (void)dst;
            throw std::runtime_error("OpenCV or FastCV backend not available");
#endif
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
            throw std::runtime_error("Unsupported image format. Only YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, RAW8, NV12, GRAY8 are supported");
            break;
    }

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    (void)warpSrcSpecs;
#endif
}

}  // namespace dai::impl

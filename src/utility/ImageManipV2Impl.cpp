#include "depthai/utility/ImageManipV2Impl.hpp"

#include <stdexcept>

#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"

#if defined(WIN32) || defined(_WIN32)
    #define _RESTRICT
#else
    #define _RESTRICT __restrict__
#endif

#define UNUSED(x) (void)(x)

dai::impl::AEEResult dai::impl::manipGetSrcMask(const uint32_t width,
                                                const uint32_t height,
                                                const float* corners,
                                                const uint32_t cornersLen,
                                                float minx,
                                                float maxx,
                                                float miny,
                                                float maxy,
                                                bool init0,
                                                uint8_t* mask,
                                                const uint32_t maskLen) {
    if(maskLen < height * width) return AEE_EBADPARM;
    const uint32_t numBoxes = cornersLen / (4 * 2);
    const float p11x = corners[0];
    const float p11y = corners[1];
    const float p12x = corners[2];
    const float p12y = corners[3];
    /*const float p13x = corners[4];*/
    /*const float p13y = corners[5];*/
    const float p14x = corners[6];
    const float p14y = corners[7];

    {
        const uint32_t iminx = std::max((uint32_t)floorf(minx), 0U);
        const uint32_t imaxx = std::min((uint32_t)ceilf(maxx), width);
        const uint32_t iminy = std::max((uint32_t)floorf(miny), 0U);
        const uint32_t imaxy = std::min((uint32_t)ceilf(maxy), height);

        if(init0) {
            memset(mask, 0, height * width * sizeof(uint8_t));
        }
        for(uint32_t i = iminy; i < imaxy; ++i) {
            const uint32_t lineStart = i * width;
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
            for(uint32_t j = lineStart + iminx; j < lineStart + imaxx; ++j) {
                const float x = j - lineStart;
                const float y = i;
                const float apx = x - p11x;
                const float apy = y - p11y;
                const float abx = p12x - p11x;
                const float aby = p12y - p11y;
                const float adx = p14x - p11x;
                const float ady = p14y - p11y;
                // (0 < AP . AB < AB . AB) and (0 < AP . AD < AD . AD)
                mask[j] = 0 <= apx * abx + apy * aby && apx * abx + apy * aby < abx * abx + aby * aby && 0 <= apx * adx + apy * ady
                          && apx * adx + apy * ady < adx * adx + ady * ady;
            }
        }
    }
    for(uint32_t b = 1; b < numBoxes; ++b) {
        const float pi1x = corners[0];
        const float pi1y = corners[1];
        const float pi2x = corners[2];
        const float pi2y = corners[3];
        /*const float pi3x = corners[4];*/
        /*const float pi3y = corners[5];*/
        const float pi4x = corners[6];
        const float pi4y = corners[7];

        const uint32_t iminx = std::max((uint32_t)floorf(minx), 0U);
        const uint32_t imaxx = std::min((uint32_t)ceilf(maxx), width);
        const uint32_t iminy = std::max((uint32_t)floorf(miny), 0U);
        const uint32_t imaxy = std::min((uint32_t)ceilf(maxy), height);

        for(uint32_t i = iminy; i < imaxy; ++i) {
            const uint32_t lineStart = i * width;
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
            for(uint32_t j = lineStart + iminx; j < lineStart + imaxx; ++j) {
                const float x = j - lineStart;
                const float y = i;
                const float apx = x - pi1x;
                const float apy = y - pi1y;
                const float abx = pi2x - pi1x;
                const float aby = pi2y - pi1y;
                const float adx = pi4x - pi1x;
                const float ady = pi4y - pi1y;
                // (0 < AP . AB < AB . AB) and (0 < AP . AD < AD . AD)
                mask[j] &= 0 < apx * abx + apy * aby && apx * abx + apy * aby < abx * abx + aby * aby && 0 < apx * adx + apy * ady
                           && apx * adx + apy * ady < adx * adx + ady * ady;
            }
        }
    }

    return AEE_SUCCESS;
}

dai::impl::AEEResult dai::impl::manipGetRemap3x3(const uint32_t inWidth,
                                                 const uint32_t inHeight,
                                                 const uint32_t outWidth,
                                                 const uint32_t outHeight,
                                                 const float* matrix,
                                                 const uint32_t matrixLen,
                                                 const uint8_t* _RESTRICT srcMask,
                                                 const uint32_t srcMaskLen,
                                                 const uint32_t minx,
                                                 const uint32_t maxx,
                                                 const uint32_t miny,
                                                 const uint32_t maxy,
                                                 float* _RESTRICT mapX,
                                                 const uint32_t mapXLen,
                                                 float* _RESTRICT mapY,
                                                 const uint32_t mapYLen,
                                                 uint8_t* _RESTRICT dstMask,
                                                 const uint32_t dstMaskLen) {
    if(mapXLen != outWidth * outHeight || mapYLen != outWidth * outHeight || matrixLen != 9 || srcMaskLen != inWidth * inHeight
       || dstMaskLen != outWidth * outHeight) {
        return AEE_EBADPARM;
    }
/**
 * 0 1 2   0
 * 3 4 5 x 1
 * 6 7 8   2
 */
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
    for(uint32_t i = 0; i < outWidth * outHeight; ++i) {
        uint32_t x = i % outWidth;
        uint32_t y = i / outWidth;

        float tX = matrix[0] * x + matrix[1] * y + matrix[2];
        float tY = matrix[3] * x + matrix[4] * y + matrix[5];
        float q = matrix[6] * x + matrix[7] * y + matrix[8];

        mapX[i] = tX / q;
        mapY[i] = tY / q;

        // Remap if outside of the mask
        mapX[i] = mapX[i] < minx ? minx : mapX[i];
        mapX[i] = mapX[i] >= maxx ? maxx - 1 : mapX[i];
        mapY[i] = mapY[i] < miny ? miny : mapY[i];
        mapY[i] = mapY[i] >= maxy ? maxy - 1 : mapY[i];

        const uint32_t idx = (uint32_t)roundf(mapY[i]) * inWidth + (uint32_t)roundf(mapX[i]);
        dstMask[i] = 0x1 & srcMask[idx];
    }
    return AEE_SUCCESS;
}

dai::impl::AEEResult dai::impl::subsampleMap2x2(const uint32_t width,
                                                const uint32_t height,
                                                const float* _RESTRICT mapX,
                                                const uint32_t mapXLen,
                                                const float* _RESTRICT mapY,
                                                const uint32_t mapYLen,
                                                const uint8_t* _RESTRICT dstMask,
                                                const uint32_t dstMaskLen,
                                                float* _RESTRICT mapXss,
                                                const uint32_t mapXssLen,
                                                float* _RESTRICT mapYss,
                                                const uint32_t mapYssLen,
                                                uint8_t* _RESTRICT dstMaskss,
                                                const uint32_t dstMaskssLen) {
    if(mapXLen != width * height || mapYLen != width * height || dstMaskLen != width * height || mapXssLen != width * height / 4
       || mapYssLen != width * height / 4 || dstMaskssLen != width * height / 4 || width % 2 != 0 || height % 2 != 0) {
        return AEE_EBADPARM;
    }
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
    for(uint32_t idx = 0; idx < width * height / 4; ++idx) {
        const uint32_t i = idx / (width / 2);
        const uint32_t j = idx % (width / 2);
        const uint32_t idx1 = 2 * i + 2 * j;
        const uint32_t idx2 = 2 * i + 2 * j + 1;
        const uint32_t idx3 = 2 * i + 1 + 2 * j;
        const uint32_t idx4 = 2 * i + 1 + 2 * j + 1;
        mapXss[idx] = (mapX[idx1] + mapX[idx2] + mapX[idx3] + mapX[idx4]) / (4.0f * 2.0f);
        mapYss[idx] = (mapY[idx1] + mapY[idx2] + mapY[idx3] + mapY[idx4]) / (4.0f * 2.0f);
        dstMaskss[idx] = dstMask[idx1] | dstMask[idx2] | dstMask[idx3] | dstMask[idx4];
    }
    return AEE_SUCCESS;
}

dai::impl::AEEResult dai::impl::remapImage(const uint8_t* _RESTRICT inData,
                                           const uint32_t inDataLen,
                                           const float* _RESTRICT mapX,
                                           const uint32_t mapXLen,
                                           const float* _RESTRICT mapY,
                                           const uint32_t mapYLen,
                                           const uint8_t* _RESTRICT dstMask,
                                           const uint32_t dstMaskLen,
                                           const uint16_t numChannels,
                                           const uint32_t inWidth,
                                           const uint32_t inHeight,
                                           const uint32_t inStride,
                                           const uint32_t outWidth,
                                           const uint32_t outHeight,
                                           const uint32_t outStride,
                                           uint8_t* _RESTRICT outData,
                                           const uint32_t outDataLen) {
    UNUSED(inDataLen);
    UNUSED(outDataLen);
    if(mapXLen != mapYLen || mapXLen != outWidth * outHeight || dstMaskLen != outWidth * outHeight || numChannels == 0 || numChannels > 3) return AEE_EBADPARM;

    for(uint32_t i = 0; i < outHeight; ++i) {
        const uint32_t lineStart = i * outStride;
        switch(numChannels) {
            case 1:
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const int x1 = floorf(x);
                    const int y1 = floorf(y);
                    const int x2 = clampi(x1 + 1, 0, inWidth - 1);
                    const int y2 = clampi(y1 + 1, 0, outHeight - 1);
                    const float fx = x - x1;
                    const float fy = y - y1;
                    const uint32_t q11i = y1 * inStride + numChannels * x1;
                    const uint32_t q21i = y1 * inStride + numChannels * x2;
                    const uint32_t q12i = y2 * inStride + numChannels * x1;
                    const uint32_t q22i = y2 * inStride + numChannels * x2;

                    {
                        const float r1 = inData[q11i + 0] * (1 - fx) + inData[q21i + 0] * fx;
                        const float r2 = inData[q12i + 0] * (1 - fx) + inData[q22i + 0] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 0] = dstMask[mapIdx] * pi;
                    }
                }
                break;
            case 2:
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const int x1 = floorf(x);
                    const int y1 = floorf(y);
                    const int x2 = clampi(x1 + 1, 0, inWidth - 1);
                    const int y2 = clampi(y1 + 1, 0, outHeight - 1);
                    const float fx = x - x1;
                    const float fy = y - y1;
                    const uint32_t q11i = y1 * inStride + numChannels * x1;
                    const uint32_t q21i = y1 * inStride + numChannels * x2;
                    const uint32_t q12i = y2 * inStride + numChannels * x1;
                    const uint32_t q22i = y2 * inStride + numChannels * x2;

                    {
                        const float r1 = inData[q11i + 0] * (1 - fx) + inData[q21i + 0] * fx;
                        const float r2 = inData[q12i + 0] * (1 - fx) + inData[q22i + 0] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 0] = dstMask[mapIdx] * pi;
                    }
                    {
                        const float r1 = inData[q11i + 1] * (1 - fx) + inData[q21i + 1] * fx;
                        const float r2 = inData[q12i + 1] * (1 - fx) + inData[q22i + 1] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 1] = dstMask[mapIdx] * pi;
                    }
                }
                break;
            case 3:
#ifdef __clang__
    #pragma clang loop vectorize(enable) interleave(enable) unroll_count(4) distribute(enable)
#else
    #pragma GCC unroll 4
    #pragma GCC ivdep
#endif
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const int x1 = floorf(x);
                    const int y1 = floorf(y);
                    const int x2 = clampi(x1 + 1, 0, inWidth - 1);
                    const int y2 = clampi(y1 + 1, 0, outHeight - 1);
                    const float fx = x - x1;
                    const float fy = y - y1;
                    const uint32_t q11i = y1 * inStride + numChannels * x1;
                    const uint32_t q21i = y1 * inStride + numChannels * x2;
                    const uint32_t q12i = y2 * inStride + numChannels * x1;
                    const uint32_t q22i = y2 * inStride + numChannels * x2;

                    {
                        const float r1 = inData[q11i + 0] * (1 - fx) + inData[q21i + 0] * fx;
                        const float r2 = inData[q12i + 0] * (1 - fx) + inData[q22i + 0] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 0] = dstMask[mapIdx] * pi;
                    }
                    {
                        const float r1 = inData[q11i + 1] * (1 - fx) + inData[q21i + 1] * fx;
                        const float r2 = inData[q12i + 1] * (1 - fx) + inData[q22i + 1] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 1] = dstMask[mapIdx] * pi;
                    }
                    {
                        const float r1 = inData[q11i + 2] * (1 - fx) + inData[q21i + 2] * fx;
                        const float r2 = inData[q12i + 2] * (1 - fx) + inData[q22i + 2] * fx;
                        const float p = r1 * (1 - fy) + r2 * fy;
                        const uint8_t pi = clampi(roundf(p), 0, 255);
                        outData[idx + 2] = dstMask[mapIdx] * pi;
                    }
                }
                break;
            default:
                return AEE_EBADPARM;
        }
    }
    return AEE_SUCCESS;
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
            throw std::runtime_error("Frame type " + std::to_string((int)srcSpecs.type) + " not supported");
            break;
    }
    return specs;
}

dai::impl::FrameSpecs dai::impl::getCcDstFrameSpecs(FrameSpecs srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to) {
    FrameSpecs specs;
    if(from == to) return srcSpecs;
    specs.width = srcSpecs.width;
    specs.height = srcSpecs.height;
    specs.p1Offset = 0;
    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = ALIGN_UP(specs.width * 3, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset;
            specs.p3Stride = 0;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p3Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p2Stride * (specs.height / 2), PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
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
            throw std::runtime_error("Frame type not supported");
            break;
    }
    return specs;
}

bool dai::impl::isTypeSupportedL(dai::ImgFrame::Type type) {
    using ImgType = dai::ImgFrame::Type;
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i;
}

bool dai::impl::isTypeSupportedC(dai::ImgFrame::Type type) {
    using ImgType = dai::ImgFrame::Type;
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i || type == ImgType::RGB888p
           || type == ImgType::BGR888p || type == ImgType::YUV420p || type == ImgType::NV12;
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

std::array<std::array<float, 2>, 4> dai::impl::getOuterRotatedRect(const std::vector<std::array<float, 2>>& points) {
    auto hull = getHull(points);
    float minArea = std::numeric_limits<float>::max();
    std::array<std::array<float, 2>, 4> minAreaPoints;

    for(size_t i = 1; i < hull.size(); ++i) {
        std::array<float, 2> vec = {hull[i][0] - hull[i - 1][0], hull[i][1] - hull[i - 1][1]};
        std::array<float, 2> vecOrth = {-vec[1], vec[0]};
        std::array<std::array<float, 2>, 2> mat = {{{vec[0], vecOrth[0]}, {vec[1], vecOrth[1]}}};
        std::array<std::array<float, 2>, 2> matInv = getInverse(mat);

        std::vector<std::array<float, 2>> rotatedHull;
        for(const auto& pt : hull) {
            float newX = matInv[0][0] * pt[0] + matInv[0][1] * pt[1];
            float newY = matInv[1][0] * pt[0] + matInv[1][1] * pt[1];
            rotatedHull.push_back({newX, newY});
        }

        const auto [minx, maxx, miny, maxy] = getOuterRect(rotatedHull);
        float area = (maxx - minx) * (maxy - miny);

        if(area < minArea) {
            minArea = area;
            std::array<std::array<float, 2>, 4> rectPoints = {{{minx, miny}, {maxx, miny}, {maxx, maxy}, {minx, maxy}}};
            for(auto i = 0U; i < rectPoints.size(); ++i) {
                auto& pt = rectPoints[i];
                float origX = mat[0][0] * pt[0] + mat[0][1] * pt[1];
                float origY = mat[1][0] * pt[0] + mat[1][1] * pt[1];
                minAreaPoints[i] = {origX, origY};
            }
        }
    }

    return minAreaPoints;
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

void dai::impl::getOutputSizeFromCorners(const std::array<std::array<float, 2>, 4>& corners, const bool center, const std::array<std::array<float, 3>, 3> transformInv, const uint32_t srcWidth, const uint32_t srcHeight, uint32_t& outputWidth, uint32_t& outputHeight) {
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
    std::array<std::array<float, 2>, 4> srcInnerCorners = {{matvecmul(transformInv, innerCorners[0]), matvecmul(transformInv, innerCorners[1]), matvecmul(transformInv, innerCorners[2]), matvecmul(transformInv, innerCorners[3])}};
    std::array<std::array<float, 2>, 4> srcOuterCorners = {{matvecmul(transformInv, outerCorners[0]), matvecmul(transformInv, outerCorners[1]), matvecmul(transformInv, outerCorners[2]), matvecmul(transformInv, outerCorners[3])}};

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
    imageCorners = getOuterRotatedRect(
        {matvecmul(mat, imageCorners[0]), matvecmul(mat, imageCorners[1]), matvecmul(mat, imageCorners[2]), matvecmul(mat, imageCorners[3])});
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
    switch(type) {
        case ImgFrame::Type::YUV420p:
            return ALIGN_UP(ALIGN_UP(width, 8) * height, PLANE_ALIGNMENT) + ALIGN_UP(ALIGN_UP(width / 2, 8) * (height / 2), PLANE_ALIGNMENT)
                   + ALIGN_UP(width / 2, 8) * (height / 2);
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            return 2 * ALIGN_UP(ALIGN_UP(width, 8) * height, PLANE_ALIGNMENT) + ALIGN_UP(width, 8) * height;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            return ALIGN_UP(3 * width, 8) * height;
        case ImgFrame::Type::NV12:
            return ALIGN_UP(ALIGN_UP(width, 8) * height, PLANE_ALIGNMENT) + ALIGN_UP(width, 8) * (height / 2);
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
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
    return 0;
}
dai::RotatedRect dai::impl::getRotatedRectFromPoints(const std::vector<std::array<float, 2>>& points) {
    auto rrCorners = impl::getOuterRotatedRect(points);
    dai::RotatedRect rect;
    rect.size.width = std::sqrt(std::pow(rrCorners[1][0] - rrCorners[0][0], 2) + std::pow(rrCorners[1][1] - rrCorners[0][1], 2));
    rect.size.height = std::sqrt(std::pow(rrCorners[2][0] - rrCorners[1][0], 2) + std::pow(rrCorners[2][1] - rrCorners[1][1], 2));
    rect.center.x = (rrCorners[0][0] + rrCorners[1][0] + rrCorners[2][0] + rrCorners[3][0]) / 4.0f;
    rect.center.y = (rrCorners[0][1] + rrCorners[1][1] + rrCorners[2][1] + rrCorners[3][1]) / 4.0f;
    rect.angle = std::atan2(rrCorners[1][1] - rrCorners[0][1], rrCorners[1][0] - rrCorners[0][0]) * 180.0f / (float)M_PI;
    return rect;
}

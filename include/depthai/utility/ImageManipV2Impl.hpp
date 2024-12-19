#pragma once
#define _USE_MATH_DEFINES

#include <spdlog/async_logger.h>
#include <stdint.h>

#include <cmath>
#include <depthai/pipeline/datatype/ImageManipConfigV2.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <sstream>

#include "depthai/common/RotatedRect.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core/base.hpp>
    #include <opencv2/core/types.hpp>
#endif

#define PLANE_ALIGNMENT 128

#if defined(WIN32) || defined(_WIN32)
    #define _RESTRICT
#else
    #define _RESTRICT __restrict__
#endif

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #define DEPTHAI_IMAGEMANIPV2_OPENCV 1
    #include <opencv2/opencv.hpp>
#endif

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace dai {
namespace impl {

class _ImageManipMemory {
    std::vector<uint8_t> _data;

   public:
    _ImageManipMemory() = default;
    _ImageManipMemory(size_t size) : _data(size) {}
    uint8_t* data() {
        return _data.data();
    }
    const uint8_t* data() const {
        return _data.data();
    }
    size_t size() const {
        return _data.size();
    }
    span<uint8_t> getData() {
        return span(data(), data() + size());
    }
    span<const uint8_t> getData() const {
        return span(data(), data() + size());
    }
};

template <typename T>
class _ImageManipBuffer {
    std::vector<T> _data;

   public:
    _ImageManipBuffer() = default;
    _ImageManipBuffer(size_t size) : _data(size) {}
    T* data() {
        return _data.data();
    }
    const T* data() const {
        return _data.data();
    }
    size_t size() const {
        return _data.size();
    }
    span<T> getData() {
        return span(data(), data() + size());
    }
    span<const T> getData() const {
        return span(data(), data() + size());
    }
};

struct FrameSpecs {
    uint32_t width;
    uint32_t height;
    uint32_t p1Offset;
    uint32_t p2Offset;
    uint32_t p3Offset;
    uint32_t p1Stride;
    uint32_t p2Stride;
    uint32_t p3Stride;
};

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class Warp {
    using Container = std::vector<ManipOp>;

    std::shared_ptr<spdlog::async_logger> logger;

    std::array<std::array<float, 3>, 3> matrix;
    ImageManipOpsBase<Container>::Background background = ImageManipOpsBase<Container>::Background::COLOR;
    uint8_t backgroundColor[3] = {0, 0, 0};

    std::shared_ptr<ImageManipBuffer<float>> mapX;
    std::shared_ptr<ImageManipBuffer<float>> mapY;
    std::shared_ptr<ImageManipBuffer<uint8_t>> srcMask;
    std::shared_ptr<ImageManipBuffer<uint8_t>> dstMask;
    std::shared_ptr<ImageManipBuffer<float>> mapXss;
    std::shared_ptr<ImageManipBuffer<float>> mapYss;
    std::shared_ptr<ImageManipBuffer<uint8_t>> srcMaskss;
    std::shared_ptr<ImageManipBuffer<uint8_t>> dstMaskss;

    std::shared_ptr<ImageManipBuffer<uint32_t>> fastCvBorder;

    ImgFrame::Type type;
    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;

    size_t sourceMinX;
    size_t sourceMinY;
    size_t sourceMaxX;
    size_t sourceMaxY;

    void transform(const uint8_t* src,
                   uint8_t* dst,
                   const size_t srcWidth,
                   const size_t srcHeight,
                   const size_t srcStride,
                   const size_t dstWidth,
                   const size_t dstHeight,
                   const size_t dstStride,
                   const uint16_t numChannels,
                   const std::array<std::array<float, 3>, 3> matrix,
                   const std::vector<uint8_t>& backgroundColor);

   public:
    Warp() = default;
    Warp(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    void build(const FrameSpecs srcFrameSpecs,
               const FrameSpecs dstFrameSpecs,
               const ImgFrame::Type type,
               const std::array<std::array<float, 3>, 3> matrix,
               std::vector<std::array<std::array<float, 2>, 4>> srcCorners);

    void apply(const span<const uint8_t> src, span<uint8_t> dst);

    Warp& setBackgroundColor(uint8_t r, uint8_t g, uint8_t b);
};

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class ColorChange {
    std::shared_ptr<spdlog::async_logger> logger;

    std::shared_ptr<ImageManipData> ccAuxFrame;
    ImgFrame::Type from;
    ImgFrame::Type to;

    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;

    bool colorConvertToRGB888i(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToBGR888p(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToRGB888p(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToBGR888i(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToNV12(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToYUV420p(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvertToGRAY8(span<const uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from);
    bool colorConvert(
        const span<uint8_t> inputFrame, span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, ImgFrame::Type from, ImgFrame::Type to);

   public:
    ColorChange() = default;
    ColorChange(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    void build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo);

    void apply(const span<const uint8_t> src, span<uint8_t> dst);
};

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class ImageManipOperations {
    using Container = std::vector<ManipOp>;

    static constexpr uint8_t MODE_CONVERT = 1;
    static constexpr uint8_t MODE_COLORMAP = 1 << 1;
    static constexpr uint8_t MODE_WARP = 1 << 2;

    uint8_t mode = 0;
    std::string prevConfig;

    std::vector<ManipOp> outputOps;

    std::array<std::array<float, 3>, 3> matrix{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> matrixInv{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::vector<std::array<std::array<float, 2>, 4>> srcCorners;
    ImageManipOpsBase<Container> base;
    ImgFrame::Type outputFrameType;
    ImgFrame::Type type;
    ImgFrame::Type inType;

    bool convertInput = false;
    bool convertOnly = false;

    std::shared_ptr<ImageManipData> colormapFrame;
    std::shared_ptr<ImageManipData> convertedFrame;
    std::shared_ptr<ImageManipData> warpedFrame;

    std::shared_ptr<spdlog::async_logger> logger;

    FrameSpecs srcSpecs;

    ColorChange<ImageManipBuffer, ImageManipData> preprocCc;
    Warp<ImageManipBuffer, ImageManipData> warpEngine;
    ColorChange<ImageManipBuffer, ImageManipData> clrChange;

   public:
    ImageManipOperations(std::shared_ptr<spdlog::async_logger> logger = nullptr) : logger(logger) {
        preprocCc.setLogger(logger);
        warpEngine.setLogger(logger);
        clrChange.setLogger(logger);
    }

    void init();

    ImageManipOperations& build(const ImageManipOpsBase<Container>& base, ImgFrame::Type outputFrameType, FrameSpecs srcFrameSpecs, ImgFrame::Type type);

    bool apply(const std::shared_ptr<Memory> src, span<uint8_t> dst);

    size_t getOutputSize() const;
    size_t getOutputWidth() const;
    size_t getOutputHeight() const;
    size_t getOutputStride(uint8_t plane = 0) const;
    FrameSpecs getOutputFrameSpecs(ImgFrame::Type type) const;
    std::vector<RotatedRect> getSrcCrops() const;
    std::array<std::array<float, 3>, 3> getMatrix() const;

    std::string toString() const;
};

FrameSpecs getSrcFrameSpecs(dai::ImgFrame::Specs srcSpecs);
size_t getAlignedOutputFrameSize(ImgFrame::Type type, size_t width, size_t height);

}  // namespace impl
}  // namespace dai

namespace dai {
namespace impl {

constexpr ImgFrame::Type VALID_TYPE_COLOR = ImgFrame::Type::RGB888i;
constexpr ImgFrame::Type VALID_TYPE_GRAY = ImgFrame::Type::GRAY8;

#ifndef ALIGN_UP
template <typename T>
constexpr T ALIGN_UP(T value, std::size_t alignment) {
    return (value + (alignment - 1)) & ~(alignment - 1);
}
#endif

FrameSpecs getSrcFrameSpecs(dai::ImgFrame::Specs srcSpecs);
FrameSpecs getCcDstFrameSpecs(FrameSpecs srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to);

static inline int clampi(int val, int minv, int maxv) {
    // return val < minv ? minv : (val > maxv ? maxv : val);
    return std::clamp(val, minv, maxv);
}
static inline float clampf(float val, float minv, float maxv) {
    // return val < minv ? minv : (val > maxv ? maxv : val);
    return std::clamp(val, minv, maxv);
}

bool isTypeSupportedL(dai::ImgFrame::Type type);
bool isTypeSupportedC(dai::ImgFrame::Type type);

bool getFrameTypeInfo(dai::ImgFrame::Type outFrameType, int& outNumPlanes, float& outBpp);

//-----------------------------
//--- Frame type conversion ---
//-----------------------------

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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToRGB888p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame.data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame.data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame.data() + dstSpecs.p1Offset);
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
                                outputFrame.data() + dstSpecs.p1Offset,
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
                                outputFrame.data() + dstSpecs.p2Offset,
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
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
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
                                outputFrame.data() + dstSpecs.p1Offset,
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
                                outputFrame.data() + dstSpecs.p2Offset,
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
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
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
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                    outputFrame[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame[p3Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToBGR888p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame.data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame.data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame.data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src, src + inputSize, outputFrame.data());
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
                                outputFrame.data() + dstSpecs.p1Offset,
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
                                outputFrame.data() + dstSpecs.p2Offset,
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
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
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
                                outputFrame.data() + dstSpecs.p1Offset,
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
                                outputFrame.data() + dstSpecs.p2Offset,
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
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
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
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
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
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                    outputFrame[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame[p3Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToRGB888i(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888i;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

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
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
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
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame[dstPos + 0] = src[srcPos + 2];
                    outputFrame[dstPos + 1] = src[srcPos + 1];
                    outputFrame[dstPos + 2] = src[srcPos + 0];
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
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                             outputFrame.data() + dstSpecs.p1Offset,
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
                    outputFrame[pos + 0] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                    outputFrame[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame[pos + 2] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToBGR888i(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888i;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);
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
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
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
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame[dstPos + 0] = src[srcPos + 2];
                    outputFrame[dstPos + 1] = src[srcPos + 1];
                    outputFrame[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
            std::copy(src, src + inputSize, outputFrame.data());
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
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                    outputFrame[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    outputFrame[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToNV12(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::NV12;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
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
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12:
            std::copy(src, src + inputSize, outputFrame.data());
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
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelCombine2Planesu8(src + srcSpecs.p2Offset,
                                       srcSpecs.width / 2,
                                       srcSpecs.height / 2,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame.data() + dstSpecs.p2Offset,
                                       dstSpecs.p2Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat frameUV(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC2, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            cv::merge(channels, frameUV);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToYUV420p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::YUV420p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
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
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
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
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
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
                                outputFrame.data() + dstSpecs.p1Offset,
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
                                outputFrame.data() + dstSpecs.p2Offset,
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
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(frameUV, channels);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p:
            std::copy(src, src + inputSize, outputFrame.data());
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToGRAY8(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::GRAY8;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            // Convert to grayscale
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxRGB, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToGrayu8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(frameRGB, gray, cv::COLOR_RGB2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                    ccAuxFrame->data()[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    ccAuxFrame->data()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    ccAuxFrame->data()[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                }
            }
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame.data());
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void ColorChange<ImageManipBuffer, ImageManipData>::build(const FrameSpecs srcFrameSpecs,
                                                          const FrameSpecs dstFrameSpecs,
                                                          const ImgFrame::Type typeFrom,
                                                          const ImgFrame::Type typeTo) {
    from = typeFrom;
    to = typeTo;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;
    size_t newAuxFrameSize = srcSpecs.height * ALIGN_UP(3 * srcSpecs.width, 8);
    if(!ccAuxFrame || ccAuxFrame->size() < newAuxFrameSize) ccAuxFrame = std::make_shared<ImageManipData>(newAuxFrameSize);
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void ColorChange<ImageManipBuffer, ImageManipData>::apply(span<const uint8_t> src, span<uint8_t> dst) {
    float bpp;
    int numPlanes;
    getFrameTypeInfo(to, numPlanes, bpp);

    // logger->debug("From {} ({}):\n\t{}x{}\n\tstride {} {} {}\n\toffset {} {} {}\n\ttotal size {}\n",
    //               (int)from,
    //               (void*)src.data(),
    //               (int)srcSpecs.width,
    //               (int)srcSpecs.height,
    //               (int)srcSpecs.p1Stride,
    //               (int)srcSpecs.p2Stride,
    //               (int)srcSpecs.p3Stride,
    //               (int)srcSpecs.p1Offset,
    //               (int)srcSpecs.p2Offset,
    //               (int)srcSpecs.p3Offset,
    //               (int)src.size());
    // logger->debug("To {} ({}):\n\t{}x{}\n\tstride {} {} {}\n\toffset {} {} {}\n\ttotal size {}\n",
    //               (int)to,
    //               (void*)dst.data(),
    //               (int)dstSpecs.width,
    //               (int)dstSpecs.height,
    //               (int)dstSpecs.p1Stride,
    //               (int)dstSpecs.p2Stride,
    //               (int)dstSpecs.p3Stride,
    //               (int)dstSpecs.p1Offset,
    //               (int)dstSpecs.p2Offset,
    //               (int)dstSpecs.p3Offset,
    //               (int)dst.size());

    bool done = false;
    auto start = std::chrono::steady_clock::now();
    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
            done = colorConvertToRGB888p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::BGR888p:
            done = colorConvertToBGR888p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::RGB888i:
            done = colorConvertToRGB888i(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::BGR888i:
            done = colorConvertToBGR888i(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::NV12:
            done = colorConvertToNV12(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::YUV420p:
            done = colorConvertToYUV420p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::GRAY8:
        case dai::ImgFrame::Type::RAW8:
            done = colorConvertToGRAY8(src, dst, srcSpecs, dstSpecs, from);
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
        std::copy(src.data(), src.data() + (src.size() <= dst.size() ? src.size() : dst.size()), dst.data());
    }
}

//------------
//--- Warp ---
//------------

// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

inline bool float_eq(float a, float b) {
    return fabs(a - b) <= 1e-6f;
}

inline bool isSingleChannelu8(const std::shared_ptr<dai::ImgFrame> img) {
    return img->getType() == dai::ImgFrame::Type::GRAY8 || img->getType() == dai::ImgFrame::Type::RAW8;
}
inline bool isSingleChannelu8(const dai::ImgFrame::Type type) {
    return type == dai::ImgFrame::Type::GRAY8 || type == dai::ImgFrame::Type::RAW8;
}

template <typename T>
inline std::string getOpStr(const T& op) {
    return op.toStr();
}

template <typename C>
std::string getConfigString(const dai::ImageManipOpsBase<C>& ops) {
    std::stringstream configSS;
    const auto operations = ops.getOperations();
    for(auto i = 0U; i < operations.size(); ++i) {
        configSS << std::visit([](auto&& op) { return getOpStr(op); }, operations[i].op);
        if(i != operations.size() - 1) configSS << " ";
    }
    return configSS.str();
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
inline std::array<float, 2> matvecmul(std::array<std::array<float, 2>, 2> M, std::array<float, 2> vec) {
    auto x = M[0][0] * vec[0] + M[0][1] * vec[1];
    auto y = M[1][0] * vec[0] + M[1][1] * vec[1];
    return {x, y};
}

std::tuple<float, float, float, float> getOuterRect(const std::vector<std::array<float, 2>> points);

std::vector<std::array<float, 2>> getHull(const std::vector<std::array<float, 2>> points);

std::array<std::array<float, 2>, 2> getInverse(const std::array<std::array<float, 2>, 2> mat);

std::array<std::array<float, 3>, 3> getInverse(const std::array<std::array<float, 3>, 3>& matrix);

std::array<std::array<float, 2>, 4> getOuterRotatedRect(const std::vector<std::array<float, 2>>& points);

dai::RotatedRect getRotatedRectFromPoints(const std::vector<std::array<float, 2>>& points);

std::array<std::array<float, 3>, 3> getResizeMat(Resize o, float width, float height, uint32_t outputWidth, uint32_t outputHeight);

void getTransformImpl(const ManipOp& op,
                      std::array<std::array<float, 3>, 3>& transform,
                      std::array<std::array<float, 2>, 4>& imageCorners,
                      std::vector<std::array<std::array<float, 2>, 4>>& srcCorners,
                      uint32_t& outputWidth,
                      uint32_t& outputHeight);

template <typename C>
std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>, std::vector<std::array<std::array<float, 2>, 4>>> getTransform(
    const C& ops, uint32_t inputWidth, uint32_t inputHeight, uint32_t outputWidth, uint32_t outputHeight) {
    std::array<std::array<float, 3>, 3> transform{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 2>, 4> imageCorners{{{0, 0}, {(float)inputWidth, 0}, {(float)inputWidth, (float)inputHeight}, {0, (float)inputHeight}}};
    std::vector<std::array<std::array<float, 2>, 4>> srcCorners;
    for(const auto& op : ops) {
        getTransformImpl(op, transform, imageCorners, srcCorners, outputWidth, outputHeight);
    }
    return {transform, imageCorners, srcCorners};
}

void getOutputSizeFromCorners(const std::array<std::array<float, 2>, 4>& corners, const bool center, const std::array<std::array<float, 3>, 3> transformInv, const uint32_t srcWidth, const uint32_t srcHeight, uint32_t& outputWidth, uint32_t& outputHeight);

template <typename C>
std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>, std::vector<std::array<std::array<float, 2>, 4>>> getFullTransform(
    dai::ImageManipOpsBase<C>& base, size_t inputWidth, size_t inputHeight, dai::ImgFrame::Type type, dai::ImgFrame::Type outputFrameType, C& outputOps) {
    using namespace dai;
    using namespace dai::impl;

    outputOps.clear();

    auto operations = base.getOperations();

    auto [matrix, imageCorners, srcCorners] = getTransform(operations, inputWidth, inputHeight, base.outputWidth, base.outputHeight);

    getOutputSizeFromCorners(imageCorners, base.center, getInverse(matrix), inputWidth, inputHeight,  base.outputWidth, base.outputHeight);

    if(base.resizeMode != ImageManipOpsBase<C>::ResizeMode::NONE) {
        Resize res;
        switch(base.resizeMode) {
            case ImageManipOpsBase<C>::ResizeMode::NONE:
                break;
            case ImageManipOpsBase<C>::ResizeMode::STRETCH:
                res = Resize(base.outputWidth, base.outputHeight);
                break;
            case ImageManipOpsBase<C>::ResizeMode::LETTERBOX:
                res = Resize::fit();
                break;
            case ImageManipOpsBase<C>::ResizeMode::CENTER_CROP:
                res = Resize::fill();
                break;
        }
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        auto mat = getResizeMat(res, maxx - minx, maxy - miny, base.outputWidth, base.outputHeight);
        imageCorners = {
            {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[2])}}};
        matrix = matmul(mat, matrix);
        outputOps.emplace_back(res);
    }

    if(base.center) {
        float width = base.outputWidth;
        float height = base.outputHeight;
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        float tx = -minx + (width - (maxx - minx)) / 2;
        float ty = -miny + (height - (maxy - miny)) / 2;
        std::array<std::array<float, 3>, 3> mat = {{{1, 0, tx}, {0, 1, ty}, {0, 0, 1}}};
        imageCorners = {
            {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[3])}}};
        matrix = matmul(mat, matrix);
        outputOps.emplace_back(Translate(tx, ty));
    }

    auto matrixInv = getInverse(matrix);

    if(type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p || outputFrameType == ImgFrame::Type::NV12
       || outputFrameType == ImgFrame::Type::YUV420p) {
        base.outputWidth = base.outputWidth - (base.outputWidth % 2);
        base.outputHeight = base.outputHeight - (base.outputHeight % 2);
    }

    srcCorners.push_back({matvecmul(matrixInv, {0, 0}),
                          matvecmul(matrixInv, {(float)base.outputWidth, 0}),
                          matvecmul(matrixInv, {(float)base.outputWidth, (float)base.outputHeight}),
                          matvecmul(matrixInv, {0, (float)base.outputHeight})});

    return {matrix, imageCorners, srcCorners};
}

inline dai::ImgFrame::Type getValidType(dai::ImgFrame::Type type) {
    return isSingleChannelu8(type) ? VALID_TYPE_GRAY : VALID_TYPE_COLOR;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void ImageManipOperations<ImageManipBuffer, ImageManipData>::init() {
#ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
    #ifndef FORCE_FASTCV_ON_SPECIFIC_CORE
    const fcvOperationMode operMode = FASTCV_OP_PERFORMANCE;  // FASTCV_OP_CPU_OFFLOAD;
    // DON'T CALL fcvSetOperationMode(operMode) BEFORE zdl::SNPE::SNPEBuilder::build() !!!
    // else zdl::SNPE::SNPEBuilder::build() will SIGSEGV !!!
    int setOperMode = fcvSetOperationMode(operMode);
    FCV_IMG_DEBUG("Set operation mode to: %d with result: %d\n", operMode, setOperMode);
    #else
    ForceExecutionOnSpecificCore();
    #endif
#endif
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
ImageManipOperations<ImageManipBuffer, ImageManipData>& ImageManipOperations<ImageManipBuffer, ImageManipData>::build(
    const ImageManipOpsBase<Container>& newBase, ImgFrame::Type outType, FrameSpecs srcFrameSpecs, ImgFrame::Type inFrameType) {
    const auto newCfgStr = getConfigString(newBase);
    if(newCfgStr == prevConfig && outType == outputFrameType && srcFrameSpecs.width == srcSpecs.width && srcFrameSpecs.height == srcSpecs.height
       && inFrameType == inType)
        return *this;
    prevConfig = newCfgStr;
    outputOps.clear();

    if(newBase.hasWarp(srcFrameSpecs.width, srcFrameSpecs.height)) mode = mode | MODE_WARP;
    if(newBase.colormap != Colormap::NONE && isSingleChannelu8(inFrameType)) mode = mode | MODE_COLORMAP;
    if(outType != ImgFrame::Type::NONE && outType != inFrameType) mode = mode | MODE_CONVERT;

    assert(inFrameType != ImgFrame::Type::NONE);
    base = newBase;
    outputFrameType = outType;
    if(outType == ImgFrame::Type::NONE) {
        if(base.colormap != Colormap::NONE) outputFrameType = VALID_TYPE_COLOR;
        else outputFrameType = inFrameType;
    }
    inType = inFrameType;
    type = inType;
    srcSpecs = srcFrameSpecs;

    if(!isSingleChannelu8(inType) && base.colormap != Colormap::NONE) {
        if(logger) logger->warn("ImageManip | Colormap can only be applied to single channel images, ignoring colormap");
        base.colormap = Colormap::NONE;
    }

    if(mode == 0) {
        return *this;
    } else if(mode == MODE_CONVERT) {
        auto ccDstSpecs = getCcDstFrameSpecs(srcSpecs, inType, outputFrameType);
        preprocCc.build(srcSpecs, ccDstSpecs, inType, outputFrameType);
    } else {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
        if(!isTypeSupportedL(inType)) {
#else
        if(!isTypeSupportedC(inType)) {
#endif
            auto color = getValidType(inType);
            auto ccDstSpecs = getCcDstFrameSpecs(srcSpecs, inType, color);
            preprocCc.build(srcSpecs, ccDstSpecs, inType, color);
            srcSpecs = ccDstSpecs;
            type = color;
            convertInput = true;
            if(outputFrameType != color)
                mode = mode & ~MODE_CONVERT;
            else
                mode = mode | MODE_CONVERT;
        }
    }

    const uint32_t inputWidth = srcSpecs.width;
    const uint32_t inputHeight = srcSpecs.height;
    assert(inputWidth > 0 && inputHeight > 0);
    matrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    if(mode & MODE_WARP) {
        auto [matrix, imageCorners, _srcCorners] = getFullTransform<Container>(base, inputWidth, inputHeight, type, outputFrameType, outputOps);

        this->matrix = matrix;
        this->matrixInv = getInverse(matrix);
        this->srcCorners = _srcCorners;

        if(logger) {
            logger->trace("Image corners: ");
            logger->trace("|{} {}|{} {}|", imageCorners[0][0], imageCorners[0][1], imageCorners[1][0], imageCorners[1][1]);
            logger->trace("-------------");
            logger->trace("|{} {}|{} {}|", imageCorners[3][0], imageCorners[3][1], imageCorners[2][0], imageCorners[2][1]);
            logger->trace("Transformation matrix: ");
            logger->trace("|{} {} {}|", matrix[0][0], matrix[0][1], matrix[0][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrix[1][0], matrix[1][1], matrix[1][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrix[2][0], matrix[2][1], matrix[2][2]);
            logger->trace("Transformation matrix inverse: ");
            logger->trace("|{} {} {}|", matrixInv[0][0], matrixInv[0][1], matrixInv[0][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrixInv[1][0], matrixInv[1][1], matrixInv[1][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrixInv[2][0], matrixInv[2][1], matrixInv[2][2]);
        }

        warpEngine.build(srcSpecs, getOutputFrameSpecs(type), type, matrix, srcCorners);
        warpEngine.setBackgroundColor(base.backgroundR, base.backgroundG, base.backgroundB);
        clrChange.build(getOutputFrameSpecs(type), getOutputFrameSpecs(outputFrameType), type, outputFrameType);
    } else {
        base.outputWidth = inputWidth;
        base.outputHeight = inputHeight;
    }
    float bppPre, bppPost;
    int numPlanesPre, numPlanesPost;
    getFrameTypeInfo(getValidType(type), numPlanesPre, bppPre);
    getFrameTypeInfo(isSingleChannelu8(type) && base.colormap != Colormap::NONE ? VALID_TYPE_COLOR : type, numPlanesPost, bppPost);
    size_t newConvertedSize = getAlignedOutputFrameSize(type, inputWidth, inputHeight);
    size_t newColormapSize = getAlignedOutputFrameSize(type, base.outputWidth, base.outputHeight);
    size_t newWarpedSize =
        getAlignedOutputFrameSize(isSingleChannelu8(type) && base.colormap != Colormap::NONE ? VALID_TYPE_COLOR : type, base.outputWidth, base.outputHeight);

    if(!convertedFrame || convertedFrame->size() < newConvertedSize) convertedFrame = std::make_shared<ImageManipData>(newConvertedSize);
    if(!colormapFrame || colormapFrame->size() < newColormapSize) colormapFrame = std::make_shared<ImageManipData>(newColormapSize);
    if(!warpedFrame || warpedFrame->size() < newWarpedSize) warpedFrame = std::make_shared<ImageManipData>(newWarpedSize);

    return *this;
}  // namespace impl

size_t getFrameSize(const ImgFrame::Type type, const FrameSpecs& specs);

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool ImageManipOperations<ImageManipBuffer, ImageManipData>::apply(const std::shared_ptr<Memory> src, span<uint8_t> dst) {
    size_t requiredSize = getFrameSize(inType, srcSpecs);
    if(src->getSize() < requiredSize)
        throw std::runtime_error("ImageManip not built for the source image specs. Consider rebuilding with the new configuration.");
    if(mode == 0) {
        std::copy(src->getData().begin(), src->getData().end(), dst.begin());
        return true;
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(convertInput || mode == MODE_CONVERT) preprocCc.apply(src->getData(), mode == MODE_CONVERT ? dst : convertedFrame->getData());
    if(mode != MODE_CONVERT) {
        if(mode & MODE_WARP) {
            warpEngine.apply(convertInput ? convertedFrame->getData() : src->getData(),
                             base.colormap != Colormap::NONE ? colormapFrame->getData() : (type == outputFrameType ? dst : warpedFrame->getData()));
        }
        if(mode & MODE_COLORMAP) {
            uint8_t* colormapDst = outputFrameType == VALID_TYPE_COLOR ? dst.data() : warpedFrame->data();
            cv::Mat gray(base.outputWidth,
                         base.outputHeight,
                         CV_8UC1,
                         mode & MODE_WARP ? colormapFrame->data() : (convertInput ? convertedFrame->data() : src->getData().data()),
                         ALIGN_UP(base.outputWidth, 8));
            cv::Mat color(base.outputWidth, base.outputHeight, CV_8UC3, colormapDst);
            cv::ColormapTypes cvColormap = cv::COLORMAP_JET;
            switch(base.colormap) {  // TODO(asahtik): set correct stereo colormaps
                case Colormap::TURBO:
                case Colormap::STEREO_TURBO:
                    cvColormap = cv::COLORMAP_TURBO;
                    break;
                case Colormap::STEREO_JET:
                case Colormap::JET:
                case Colormap::NONE:
                    break;
            }
            cv::applyColorMap(gray, color, cvColormap);
        }
        // Change color(format) if outputFrameType is not None / the same as the current frame type and not (frame type is single channel + colormap is
        // applied
        // + output frame type is RGBi)
        if(type != outputFrameType && !(isSingleChannelu8(type) && base.colormap != Colormap::NONE && outputFrameType == VALID_TYPE_COLOR)) {
            clrChange.apply(warpedFrame->getData(), dst);
        }
    }
    return true;  // TODO(asahtik): Handle failed transformation
#else
    return false;
#endif
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData>::getOutputWidth() const {
    return base.outputWidth;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData>::getOutputHeight() const {
    return base.outputHeight;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData>::getOutputStride(uint8_t plane) const {
    if(mode == 0) return plane == 0 ? srcSpecs.p1Stride : (plane == 1 ? srcSpecs.p2Stride : (plane == 2 ? srcSpecs.p3Stride : 0));
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            return plane < 3 ? ALIGN_UP(base.outputWidth, 8) : 0;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            return plane == 0 ? ALIGN_UP(base.outputWidth * 3, 8) : 0;
        case ImgFrame::Type::NV12:
            return plane < 2 ? ALIGN_UP(base.outputWidth, 8) : 0;
        case ImgFrame::Type::YUV420p:
            return plane == 0 ? ALIGN_UP(base.outputWidth, 8) : (plane < 3 ? base.outputWidth / 2 : 0);
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
            return plane == 0 ? ALIGN_UP(base.outputWidth, 8) : 0;
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
            return 0;
    }
    return 0;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData>::getOutputSize() const {
    if(mode == 0) return 0;
    size_t size = 0;
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            size = ALIGN_UP(getOutputStride() * getOutputHeight(), PLANE_ALIGNMENT) * 3;
            break;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
            size = getOutputStride() * getOutputHeight();
            break;
        case ImgFrame::Type::NV12:
            size = ALIGN_UP(getOutputStride(0) * getOutputHeight(), PLANE_ALIGNMENT) + ALIGN_UP(getOutputStride(1) * getOutputHeight() / 2, PLANE_ALIGNMENT);
            break;
        case ImgFrame::Type::YUV420p:
            size =
                ALIGN_UP(getOutputStride(0) * getOutputHeight(), PLANE_ALIGNMENT) + ALIGN_UP(getOutputStride(1) * getOutputHeight() / 2, PLANE_ALIGNMENT) * 2;
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
            throw std::runtime_error("Output frame type not supported");
    }
    if(size == 0) throw std::runtime_error("Output size is 0");
    return size;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
FrameSpecs ImageManipOperations<ImageManipBuffer, ImageManipData>::getOutputFrameSpecs(ImgFrame::Type type) const {
    if(mode == 0) return srcSpecs;
    FrameSpecs specs;
    specs.width = base.outputWidth;
    specs.height = base.outputHeight;
    specs.p1Offset = 0;
    switch(type) {
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
std::vector<RotatedRect> ImageManipOperations<ImageManipBuffer, ImageManipData>::getSrcCrops() const {
    std::vector<RotatedRect> crops;
    for(const auto& corners : srcCorners) {
        auto rect = getRotatedRectFromPoints({corners[0], corners[1], corners[2], corners[3]});
        crops.push_back(rect);
    }
    return crops;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
std::array<std::array<float, 3>, 3> ImageManipOperations<ImageManipBuffer, ImageManipData>::getMatrix() const {
    return matrix;
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
std::string ImageManipOperations<ImageManipBuffer, ImageManipData>::toString() const {
    std::stringstream cStr;
    cStr << getConfigString(base);
    if(outputOps.size() > 0) {
        cStr << " | ";
        for(auto i = 0U; i < outputOps.size(); ++i) {
            cStr << std::visit([](auto&& op) { return getOpStr(op); }, outputOps[i].op);
            if(i != outputOps.size() - 1) cStr << " ";
        }
    }
    cStr << "; o=" << base.outputWidth << "x" << base.outputHeight;
    return cStr.str();
}

enum AEEResult { AEE_SUCCESS, AEE_EBADPARM, AEE_ERROR };

AEEResult manipGetSrcMask(const uint32_t width,
                          const uint32_t height,
                          const float* corners,
                          const uint32_t cornersLen,
                          float minx,
                          float maxx,
                          float miny,
                          float maxy,
                          bool init0,
                          uint8_t* mask,
                          const uint32_t maskLen);

AEEResult manipGetRemap3x3(const uint32_t inWidth,
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
                           const uint32_t dstMaskLen);

AEEResult subsampleMap2x2(const uint32_t width,
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
                          const uint32_t dstMaskssLen);

AEEResult remapImage(const uint8_t* _RESTRICT inData,
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
                     const uint32_t outDataLen);

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void Warp<ImageManipBuffer, ImageManipData>::build(const FrameSpecs srcFrameSpecs,
                                                   const FrameSpecs dstFrameSpecs,
                                                   const ImgFrame::Type type,
                                                   const std::array<std::array<float, 3>, 3> matrix,
                                                   std::vector<std::array<std::array<float, 2>, 4>> srcCorners) {
    this->matrix = matrix;
    this->type = type;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;

    if(!fastCvBorder || fastCvBorder->size() < dstSpecs.height * 2) fastCvBorder = std::make_shared<ImageManipBuffer<uint32_t>>(dstSpecs.height * 2);

    const uint32_t inWidth = srcFrameSpecs.width;
    const uint32_t inHeight = srcFrameSpecs.height;
    sourceMinX = 0;
    sourceMaxX = inWidth;
    sourceMinY = 0;
    sourceMaxY = inHeight;
    for(const auto& corners : srcCorners) {
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector<std::array<float, 2>>(corners.begin(), corners.end()));
        sourceMinX = std::max(sourceMinX, (size_t)std::floor(minx));
        sourceMinY = std::max(sourceMinY, (size_t)std::floor(miny));
        sourceMaxX = std::min(sourceMaxX, (size_t)std::ceil(maxx));
        sourceMaxY = std::min(sourceMaxY, (size_t)std::ceil(maxy));
    }
    if(sourceMinX >= sourceMaxX || sourceMinY >= sourceMaxY) throw std::runtime_error("Initial crop is outside the source image");

#if !DEPTHAI_IMAGEMANIPV2_OPENCV && !DEPTHAI_IMAGEMANIPV2_FASTCV || !defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && !defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    const uint32_t outWidth = dstFrameSpecs.width;
    const uint32_t outHeight = dstFrameSpecs.height;
    const auto matrixInv = getInverse(matrix);
    if(!srcMask || srcMask->size() < inWidth * inHeight) srcMask = std::make_shared<ImageManipBuffer<uint8_t>>(inWidth * inHeight);
    if(!dstMask || dstMask->size() < outWidth * outHeight) dstMask = std::make_shared<ImageManipBuffer<uint8_t>>(outWidth * outHeight);
    if(!mapX || mapX->size() < outWidth * outHeight) mapX = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
    if(!mapY || mapY->size() < outWidth * outHeight) mapY = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12) {
        if(!srcMaskss || srcMaskss->size() < inWidth * inHeight) srcMaskss = std::make_shared<ImageManipBuffer<uint8_t>>(inWidth * inHeight / 4);
        if(!dstMaskss || dstMaskss->size() < outWidth * outHeight) dstMaskss = std::make_shared<ImageManipBuffer<uint8_t>>(outWidth * outHeight / 4);
        if(!mapXss || mapXss->size() < outWidth * outHeight) mapXss = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight / 4);
        if(!mapYss || mapYss->size() < outWidth * outHeight) mapYss = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight / 4);
    }

    std::vector<float> cornersArr;
    cornersArr.reserve(srcCorners.size() * 4 * 2);
    float minx = inWidth, maxx = 0, miny = inHeight, maxy = 0;
    for(const auto& corners : srcCorners)
        for(const auto& corner : corners) {
            cornersArr.push_back(clampf(corner[0], 0.0f, (float)inWidth));
            cornersArr.push_back(clampf(corner[1], 0.0f, (float)inHeight));
            minx = std::min(minx, corner[0]);
            maxx = std::max(maxx, corner[0]);
            miny = std::min(miny, corner[1]);
            maxy = std::max(maxy, corner[1]);
        }
    minx = fmaxf(minx, 0);
    maxx = fminf(maxx, inWidth);
    miny = fmaxf(miny, 0);
    maxy = fminf(maxy, inHeight);
    std::array matrixInv1D = {matrixInv[0][0],
                              matrixInv[0][1],
                              matrixInv[0][2],
                              matrixInv[1][0],
                              matrixInv[1][1],
                              matrixInv[1][2],
                              matrixInv[2][0],
                              matrixInv[2][1],
                              matrixInv[2][2]};

    // 1. Get src mask
    AEEResult err1 = manipGetSrcMask(inWidth, inHeight, cornersArr.data(), cornersArr.size(), minx, maxx, miny, maxy, true, srcMask->data(), srcMask->size());
    if(err1 != AEE_SUCCESS) throw std::runtime_error("Failed to get src mask");
    // 2. Get transform + dst mask
    AEEResult err2 = manipGetRemap3x3(inWidth,
                                      inHeight,
                                      outWidth,
                                      outHeight,
                                      matrixInv1D.data(),
                                      9,
                                      srcMask->data(),
                                      srcMask->size(),
                                      fmaxf(minx, 0),
                                      fminf(maxx, inWidth),
                                      fmaxf(miny, 0),
                                      fminf(maxy, inHeight),
                                      mapX->data(),
                                      mapX->size(),
                                      mapY->data(),
                                      mapY->size(),
                                      dstMask->data(),
                                      dstMask->size());
    if(err2 != AEE_SUCCESS) throw std::runtime_error("Failed to get remap map");
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12) {
        assert(mapX->size() == outWidth * outHeight);
        assert(mapY->size() == outWidth * outHeight);
        assert(dstMask->size() == outWidth * outHeight);
        assert(mapXss->size() == outWidth * outHeight / 4);
        assert(mapYss->size() == outWidth * outHeight / 4);
        assert(dstMaskss->size() == outWidth * outHeight / 4);
        AEEResult err3 = subsampleMap2x2(outWidth,
                                         outHeight,
                                         mapX->data(),
                                         mapX->size(),
                                         mapY->data(),
                                         mapY->size(),
                                         dstMask->data(),
                                         dstMask->size(),
                                         mapXss->data(),
                                         mapXss->size(),
                                         mapYss->data(),
                                         mapYss->size(),
                                         dstMaskss->data(),
                                         dstMaskss->size());
        if(err3 != AEE_SUCCESS) throw std::runtime_error("Failed to subsample map");
    }
#endif
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void Warp<ImageManipBuffer, ImageManipData>::transform(const uint8_t* src,
                                                       uint8_t* dst,
                                                       const size_t srcWidth,
                                                       const size_t srcHeight,
                                                       const size_t srcStride,
                                                       const size_t dstWidth,
                                                       const size_t dstHeight,
                                                       const size_t dstStride,
                                                       const uint16_t numChannels,
                                                       const std::array<std::array<float, 3>, 3> matrix,
                                                       const std::vector<uint8_t>& background) {
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_OPENCV
    auto type = numChannels == 1 ? CV_8UC1 : (numChannels == 2 ? CV_8UC2 : CV_8UC3);
    auto bg = numChannels == 1 ? cv::Scalar(background[0])
                               : (numChannels == 2 ? cv::Scalar(background[0], background[1]) : cv::Scalar(background[0], background[1], background[2]));
    const cv::Mat cvSrc(srcHeight, srcWidth, type, const_cast<uint8_t*>(src), srcStride);
    cv::Mat cvDst(dstHeight, dstWidth, type, dst, dstStride);
#elif defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
    if(numChannels != 3 && numChannels != 1) throw std::runtime_error("Only 1 or 3 channels supported with FastCV");
    if(!((ptrdiff_t)src % 128 == 0 && (ptrdiff_t)dst % 128 == 0 && (ptrdiff_t)fastCvBorder->data() % 128 == 0 && srcStride % 8 == 0 && srcStride > 0)) {
        throw std::runtime_error("Assumptions not taken into account");
    }
#endif
    int ssF = srcSpecs.width / srcWidth;
    assert(ssF == (int)(srcSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));  // Sanity check
    if(float_eq(matrix[2][0], 0) && float_eq(matrix[2][1], 0) && float_eq(matrix[2][2], 1)) {
        // Affine transform
        float affine[6] = {matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF};
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_OPENCV
        cv::Rect roi(sourceMinX / ssF, sourceMinY / ssF, (sourceMaxX - sourceMinX) / ssF, (sourceMaxY - sourceMinY) / ssF);
        if(sourceMinX != 0 || sourceMinY != 0) {
            affine[2] = affine[0] * (float)(sourceMinX / ssF) + affine[1] * (float)(sourceMinY / ssF) + affine[2];
            affine[5] = affine[3] * (float)(sourceMinX / ssF) + affine[4] * (float)(sourceMinY / ssF) + affine[5];
        }
        cv::Mat cvAffine(2, 3, CV_32F, affine);
        cv::warpAffine(cvSrc(roi),
                       cvDst,
                       cvAffine,
                       cv::Size(dstWidth, dstHeight),
                       cv::INTER_LINEAR,
                       cv::BORDER_CONSTANT,
                       bg);  // TODO(asahtik): Add support for different border types
#elif defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
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
            fcv3ChannelTransformAffineClippedBCu8(src, srcWidth, srcHeight, srcStride, affine, dst, dstWidth, dstHeight, dstStride, fastCvBorder->data());
        }
#endif
    } else {
        // Perspective transform
        float projection[9] = {
            matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF, matrix[2][0], matrix[2][1], matrix[2][2]};
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_OPENCV
        cv::Rect roi(sourceMinX / ssF, sourceMinY / ssF, (sourceMaxX - sourceMinX) / ssF, (sourceMaxY - sourceMinY) / ssF);
        if(sourceMinX != 0 || sourceMinY != 0) {
            projection[2] = projection[0] * (float)(sourceMinX / ssF) + projection[1] * (float)(sourceMinY / ssF) + projection[2];
            projection[5] = projection[3] * (float)(sourceMinX / ssF) + projection[4] * (float)(sourceMinY / ssF) + projection[5];
            projection[8] = projection[6] * (float)(sourceMinX / ssF) + projection[7] * (float)(sourceMinY / ssF) + projection[8];
        }
        cv::Mat cvProjection(3, 3, CV_32F, projection);
        cv::warpPerspective(cvSrc(roi),
                            cvDst,
                            cvProjection,
                            cv::Size(dstWidth, dstHeight),
                            cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT,
                            bg);  // TODO(asahtik): Add support for different border types
#elif defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && DEPTHAI_IMAGEMANIPV2_FASTCV
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
#endif
    }
}

void printSpecs(spdlog::async_logger& logger, FrameSpecs specs);

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void Warp<ImageManipBuffer, ImageManipData>::apply(const span<const uint8_t> src, span<uint8_t> dst) {
    // Apply transformation multiple times depending on the image format
    switch(type) {
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            transform(src.data() + srcSpecs.p1Offset,
                      dst.data() + dstSpecs.p1Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p1Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p1Stride,
                      3,
                      matrix,
                      {backgroundColor[0], backgroundColor[1], backgroundColor[2]});
#else
            assert(mapX && mapY && dstMask && srcMask);
            assert(dstSpecs.width * dstSpecs.height * 3 <= dst.size());
            remapImage(src.data() + srcSpecs.p1Offset,
                       src.size() - srcSpecs.p1Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       3,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p1Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p1Stride,
                       dst.data() + dstSpecs.p1Offset,
                       dst.size() - dstSpecs.p1Offset);
#endif
            break;
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::RGB888p:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)

            transform(src.data() + srcSpecs.p1Offset,
                      dst.data() + dstSpecs.p1Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p1Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p1Stride,
                      1,
                      matrix,
                      {backgroundColor[0]});
            transform(src.data() + srcSpecs.p2Offset,
                      dst.data() + dstSpecs.p2Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p2Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p2Stride,
                      1,
                      matrix,
                      {backgroundColor[1]});
            transform(src.data() + srcSpecs.p3Offset,
                      dst.data() + dstSpecs.p3Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p3Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p3Stride,
                      1,
                      matrix,
                      {backgroundColor[2]});
#else
            assert(mapX && mapY && dstMask && srcMask);
            assert(dstSpecs.width * dstSpecs.height * 3 <= dst.size());
            remapImage(src.data() + srcSpecs.p1Offset,
                       src.size() - srcSpecs.p1Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p1Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p1Stride,
                       dst.data() + dstSpecs.p1Offset,
                       dst.size() - dstSpecs.p1Offset);
            remapImage(src.data() + srcSpecs.p2Offset,
                       src.size() - srcSpecs.p2Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p2Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p2Stride,
                       dst.data() + dstSpecs.p2Offset,
                       dst.size() - dstSpecs.p2Offset);
            remapImage(src.data() + srcSpecs.p3Offset,
                       src.size() - srcSpecs.p3Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p3Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p3Stride,
                       dst.data() + dstSpecs.p3Offset,
                       dst.size() - dstSpecs.p3Offset);
#endif
            break;
        case ImgFrame::Type::YUV420p:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            transform(src.data() + srcSpecs.p1Offset,
                      dst.data() + dstSpecs.p1Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p1Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p1Stride,
                      1,
                      matrix,
                      {backgroundColor[0]});
            transform(src.data() + srcSpecs.p2Offset,
                      dst.data() + dstSpecs.p2Offset,
                      srcSpecs.width / 2,
                      srcSpecs.height / 2,
                      srcSpecs.p2Stride,
                      dstSpecs.width / 2,
                      dstSpecs.height / 2,
                      dstSpecs.p2Stride,
                      1,
                      matrix,
                      {backgroundColor[1]});
            transform(src.data() + srcSpecs.p3Offset,
                      dst.data() + dstSpecs.p3Offset,
                      srcSpecs.width / 2,
                      srcSpecs.height / 2,
                      srcSpecs.p3Stride,
                      dstSpecs.width / 2,
                      dstSpecs.height / 2,
                      dstSpecs.p3Stride,
                      1,
                      matrix,
                      {backgroundColor[2]});
#else
            assert(mapX && mapY && dstMask && srcMask);
            assert(dstSpecs.width * dstSpecs.height * 3 / 2 <= dst.size());
            assert(dstSpecs.width % 2 == 0 && dstSpecs.height % 2 == 0);
            remapImage(src.data() + srcSpecs.p1Offset,
                       src.size() - srcSpecs.p1Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p1Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p1Stride,
                       dst.data() + dstSpecs.p1Offset,
                       dst.size() - dstSpecs.p1Offset);
            remapImage(src.data() + srcSpecs.p2Offset,
                       src.size() - srcSpecs.p2Offset,
                       mapXss->data(),
                       mapXss->size(),
                       mapYss->data(),
                       mapYss->size(),
                       dstMaskss->data(),
                       dstMaskss->size(),
                       1,
                       srcSpecs.width / 2,
                       srcSpecs.height / 2,
                       srcSpecs.p2Stride,
                       dstSpecs.width / 2,
                       dstSpecs.height / 2,
                       dstSpecs.p2Stride,
                       dst.data() + dstSpecs.p2Offset,
                       dst.size() - dstSpecs.p2Offset);
            remapImage(src.data() + srcSpecs.p3Offset,
                       src.size() - srcSpecs.p3Offset,
                       mapXss->data(),
                       mapXss->size(),
                       mapYss->data(),
                       mapYss->size(),
                       dstMaskss->data(),
                       dstMaskss->size(),
                       1,
                       srcSpecs.width / 2,
                       srcSpecs.height / 2,
                       srcSpecs.p3Stride,
                       dstSpecs.width / 2,
                       dstSpecs.height / 2,
                       dstSpecs.p2Stride,
                       dst.data() + dstSpecs.p3Offset,
                       dst.size() - dstSpecs.p3Offset);
#endif
            break;
        case ImgFrame::Type::NV12:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            transform(src.data() + srcSpecs.p1Offset,
                      dst.data() + dstSpecs.p1Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p1Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p1Stride,
                      1,
                      matrix,
                      {backgroundColor[0]});
            transform(src.data() + srcSpecs.p2Offset,
                      dst.data() + dstSpecs.p2Offset,
                      srcSpecs.width / 2,
                      srcSpecs.height / 2,
                      srcSpecs.p2Stride,
                      dstSpecs.width / 2,
                      dstSpecs.height / 2,
                      dstSpecs.p2Stride,
                      2,
                      matrix,
                      {backgroundColor[1], backgroundColor[2]});
#else
            assert(mapX && mapY && dstMask && srcMask);
            assert(dstSpecs.p1Stride * dstSpecs.height * 3 / 2 <= dst.size());
            assert(dstSpecs.width % 2 == 0 && dstSpecs.height % 2 == 0);
            remapImage(src.data() + srcSpecs.p1Offset,
                       src.size() - srcSpecs.p1Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p1Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p1Stride,
                       dst.data() + dstSpecs.p1Offset,
                       dst.size() - dstSpecs.p1Offset);
            remapImage(src.data() + srcSpecs.p2Offset,
                       src.size() - srcSpecs.p2Offset,
                       mapXss->data(),
                       mapXss->size(),
                       mapYss->data(),
                       mapYss->size(),
                       dstMaskss->data(),
                       dstMaskss->size(),
                       2,
                       srcSpecs.width / 2,
                       srcSpecs.height / 2,
                       srcSpecs.p2Stride,
                       dstSpecs.width / 2,
                       dstSpecs.height / 2,
                       dstSpecs.p2Stride,
                       dst.data() + dstSpecs.p2Offset,
                       dst.size() - dstSpecs.p2Offset);
#endif
            break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
#if DEPTHAI_IMAGEMANIPV2_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT) || DEPTHAI_IMAGEMANIPV2_FASTCV && defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            transform(src.data() + srcSpecs.p1Offset,
                      dst.data() + dstSpecs.p1Offset,
                      srcSpecs.width,
                      srcSpecs.height,
                      srcSpecs.p1Stride,
                      dstSpecs.width,
                      dstSpecs.height,
                      dstSpecs.p1Stride,
                      1,
                      matrix,
                      {backgroundColor[0]});
#else
            assert(mapX && mapY && dstMask && srcMask);
            assert(dstSpecs.width * dstSpecs.height <= dst.size());
            remapImage(src.data() + srcSpecs.p1Offset,
                       src.size() - srcSpecs.p1Offset,
                       mapX->data(),
                       mapX->size(),
                       mapY->data(),
                       mapY->size(),
                       dstMask->data(),
                       dstMask->size(),
                       1,
                       srcSpecs.width,
                       srcSpecs.height,
                       srcSpecs.p1Stride,
                       dstSpecs.width,
                       dstSpecs.height,
                       dstSpecs.p1Stride,
                       dst.data() + dstSpecs.p1Offset,
                       dst.size() - dstSpecs.p1Offset);
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
            throw std::runtime_error("Unsupported image format. Only YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, RAW8, NV12, GRAY8 are supported");
            break;
    }
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
Warp<ImageManipBuffer, ImageManipData>& Warp<ImageManipBuffer, ImageManipData>::setBackgroundColor(const uint8_t r, const uint8_t g, const uint8_t b) {
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
    return *this;
}

}  // namespace impl
}  // namespace dai

#undef _RESTRICT

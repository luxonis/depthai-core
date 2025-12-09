#pragma once
#include <string>
#define _USE_MATH_DEFINES

#include <spdlog/async_logger.h>
#include <stdint.h>

#include <cmath>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/properties/ImageManipProperties.hpp>
#include <sstream>

#include "depthai/common/RotatedRect.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core/base.hpp>
    #include <opencv2/core/types.hpp>
#endif

#ifndef DEPTHAI_STRIDE_ALIGNMENT
    #define DEPTHAI_STRIDE_ALIGNMENT 128
#endif
#ifndef DEPTHAI_HEIGHT_ALIGNMENT
    #define DEPTHAI_HEIGHT_ALIGNMENT 32
#endif
#ifndef DEPTHAI_PLANE_ALIGNMENT
    #define DEPTHAI_PLANE_ALIGNMENT 128 * 32
#endif

#if defined(WIN32) || defined(_WIN32)
    #define _RESTRICT
#else
    #define _RESTRICT __restrict__
#endif

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #define DEPTHAI_IMAGEMANIPV2_OPENCV 1
    #include <opencv2/opencv.hpp>
#endif
#ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
    #include <fastcv/fastcv.h>
#endif
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace dai {
namespace impl {
template <typename N, template <typename T> typename ImageManipBuffer, typename ImageManipData>
void loop(N& node,
          const ImageManipConfig& initialConfig,
          std::shared_ptr<spdlog::async_logger> logger,
          std::function<size_t(const ImageManipConfig&, const ImgFrame&)> build,
          std::function<bool(std::shared_ptr<Memory>&, std::shared_ptr<ImageManipData>)> apply,
          std::function<void(const ImgFrame&, ImgFrame&)> getFrame) {
    using namespace std::chrono;
    auto config = initialConfig;

    std::shared_ptr<ImgFrame> inImage;

    while(node.isRunning()) {
        std::shared_ptr<ImageManipConfig> pConfig;
        bool hasConfig = false;
        bool needsImage = true;
        bool skipImage = false;
        if(node.inputConfig.getWaitForMessage()) {
            pConfig = node.inputConfig.template get<ImageManipConfig>();
            hasConfig = true;
            if(inImage != nullptr && hasConfig && pConfig->getReusePreviousImage()) {
                needsImage = false;
            }
            skipImage = pConfig->getSkipCurrentImage();
        } else {
            pConfig = node.inputConfig.template tryGet<ImageManipConfig>();
            if(pConfig != nullptr) {
                hasConfig = true;
            }
        }

        if(needsImage) {
            inImage = node.inputImage.template get<ImgFrame>();
            if(inImage == nullptr) {
                logger->warn("No input image, skipping frame");
                continue;
            }
            if(!hasConfig) {
                auto _pConfig = node.inputConfig.template tryGet<ImageManipConfig>();
                if(_pConfig != nullptr) {
                    pConfig = _pConfig;
                    hasConfig = true;
                }
            }
            if(skipImage) {
                continue;
            }
        }

        // if has new config, parse and check if any changes
        if(hasConfig) {
            config = *pConfig;
        }
        if(!node.inputConfig.getWaitForMessage() && config.getReusePreviousImage()) {
            logger->warn("reusePreviousImage is only taken into account when inputConfig is synchronous");
        }

        auto startP = std::chrono::steady_clock::now();

        auto t1 = steady_clock::now();
        auto outputSize = build(config, *inImage);
        auto t2 = steady_clock::now();

        // Check the output image size requirements, and check whether pool has the size required
        if(outputSize == 0) {
            node.out.send(inImage);
        } else if((long)outputSize <= (long)node.properties.outputFrameSize) {
            auto outImage = std::make_shared<ImgFrame>();
            auto outImageData = std::make_shared<ImageManipData>(node.properties.outputFrameSize);
            outImage->data = outImageData;

            bool success = true;
            {
                auto t3 = steady_clock::now();
                success = apply(inImage->data, outImageData);
                auto t4 = steady_clock::now();

                getFrame(*inImage, *outImage);

                logger->trace("Build time: {}us, Process time: {}us, Total time: {}us, image manip id: {}",
                              duration_cast<microseconds>(t2 - t1).count(),
                              duration_cast<microseconds>(t4 - t3).count(),
                              duration_cast<microseconds>(t4 - t1).count(),
                              node.id);
            }
            if(!success) {
                logger->error("Processing failed, potentially unsupported config");
            }
            node.out.send(outImage);
        } else {
            logger->error(
                "Output image is bigger ({}B) than maximum frame size specified in properties ({}B) - skipping frame.\nPlease use the setMaxOutputFrameSize "
                "API to explicitly config the [maximum] output size.",
                outputSize,
                node.properties.outputFrameSize);
        }

        // Update previousConfig of preprocessor, to be able to check if it needs to be updated
        auto loopNanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startP).count();
        logger->trace("ImageManip | total process took {}ns ({}ms)", loopNanos, (double)loopNanos / 1e6);
    }
}

class _ImageManipMemory : public Memory {
    std::shared_ptr<std::vector<uint8_t>> _data;
    span<uint8_t> _span;
    size_t _offset = 0;

   public:
    _ImageManipMemory() = default;
    _ImageManipMemory(size_t size) {
        _data = std::make_shared<std::vector<uint8_t>>(size);
        _span = span(*_data);
    }
    _ImageManipMemory(span<uint8_t> data) : _span(data) {}
    uint8_t* data() {
        return _span.data() + _offset;
    }
    const uint8_t* data() const {
        return _span.data() + _offset;
    }
    size_t size() const {
        return _span.size() - _offset;
    }
    span<uint8_t> getData() override {
        return span(data(), data() + size());
    }
    span<const uint8_t> getData() const override {
        return span(data(), data() + size());
    }
    size_t getMaxSize() const override {
        return _span.size();
    }
    size_t getOffset() const override {
        return _offset;
    }
    void setSize(size_t size) override {
        if(size > _span.size()) {
            auto oldData = _data;
            _data = std::make_shared<std::vector<uint8_t>>(size);
            std::copy(oldData->begin(), oldData->end(), _data->begin());
            _span = span(*_data);
        } else {
            _span = _span.subspan(0, size);
        }
    }
    void setOffset(size_t offset) {
        _offset = std::min(_offset + offset, _span.size());
    }
    void shallowCopyFrom(_ImageManipMemory& other) {
        if(_data) {
            _data = other._data;
        }
        _span = other._span;
        _offset = other._offset;
    }
    std::shared_ptr<_ImageManipMemory> offset(size_t offset) {
        auto mem = std::make_shared<_ImageManipMemory>();
        mem->shallowCopyFrom(*this);
        mem->setOffset(offset);
        return mem;
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

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
class UndistortOpenCvImpl {
   public:
    enum class BuildStatus { ONE_SHOT, TWO_SHOT, NOT_USED, NOT_BUILT, ERROR };

   private:
    cv::Mat undistortMap1;
    cv::Mat undistortMap2;
    cv::Mat undistortMap1Half;
    cv::Mat undistortMap2Half;

    std::shared_ptr<spdlog::async_logger> logger;

    std::array<float, 9> cameraMatrix;
    std::array<float, 9> newCameraMatrix;
    std::vector<float> distCoeffs;
    dai::ImgFrame::Type type;
    uint32_t srcWidth;
    uint32_t srcHeight;
    uint32_t dstWidth;
    uint32_t dstHeight;

    bool validMatrix(std::array<float, 9> matrix) const;
    void initMaps(std::array<float, 9> cameraMatrix,
                  std::array<float, 9> newCameraMatrix,
                  std::vector<float> distCoeffs,
                  dai::ImgFrame::Type type,
                  uint32_t srcWidth,
                  uint32_t srcHeight,
                  uint32_t dstWidth,
                  uint32_t dstHeight);

   public:
    UndistortOpenCvImpl(std::shared_ptr<spdlog::async_logger> logger) : logger(std::move(logger)) {}
    BuildStatus build(std::array<float, 9> cameraMatrix,
                      std::array<float, 9> newCameraMatrix,
                      std::vector<float> distCoeffs,
                      dai::ImgFrame::Type type,
                      uint32_t srcWidth,
                      uint32_t srcHeight,
                      uint32_t dstWidth,
                      uint32_t dstHeight);
    void undistort(cv::Mat& src, cv::Mat& dst);
};
#endif

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class Warp {
   protected:
    using Container = std::vector<ManipOp>;

    std::shared_ptr<spdlog::async_logger> logger;

    bool isIdentityWarp() const;

   public:
    std::array<std::array<float, 3>, 3> matrix;
    ImageManipOpsBase<Container>::Background background = ImageManipOpsBase<Container>::Background::COLOR;
    uint32_t backgroundColor[3] = {0, 0, 0};
    bool enableUndistort = false;
    bool undistortOneShot = false;

    ImgFrame::Type type;
    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;

    size_t sourceMinX;
    size_t sourceMinY;
    size_t sourceMaxX;
    size_t sourceMaxY;

    Warp() = default;
    Warp(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}
    virtual ~Warp() = default;

    virtual void init(ImageManipProperties& /* properties */) {}
    virtual void build(const FrameSpecs srcFrameSpecs,
                       const FrameSpecs dstFrameSpecs,
                       const ImgFrame::Type type,
                       const std::array<std::array<float, 3>, 3> matrix,
                       std::vector<std::array<std::array<float, 2>, 4>> srcCorners) = 0;
    virtual void buildUndistort(bool enable,
                                const std::array<float, 9>& cameraMatrix,
                                const std::array<float, 9>& newCameraMatrix,
                                const std::vector<float>& distCoeffs,
                                const ImgFrame::Type type,
                                const uint32_t srcWidth,
                                const uint32_t srcHeight,
                                const uint32_t dstWidth,
                                const uint32_t dstHeight) = 0;

    virtual void apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst) = 0;

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    Warp& setBackgroundColor(uint32_t r, uint32_t g, uint32_t b);
};

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class WarpH : public Warp<ImageManipBuffer, ImageManipData> {
    std::shared_ptr<ImageManipBuffer<uint32_t>> fastCvBorder;
    std::shared_ptr<ImageManipData> auxFrame;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<UndistortOpenCvImpl> undistortImpl;
#else
    std::unique_ptr<uint32_t> dummyUndistortImpl;
#endif

    void transform(const std::shared_ptr<ImageManipData> srcData,
                   std::shared_ptr<ImageManipData> dstData,
                   const size_t srcWidth,
                   const size_t srcHeight,
                   const size_t srcStride,
                   const size_t dstWidth,
                   const size_t dstHeight,
                   const size_t dstStride,
                   const uint16_t numChannels,
                   const uint16_t bpp,
                   const std::array<std::array<float, 3>, 3> matrix,
                   const std::vector<uint32_t>& backgroundColor);

   public:
    void build(const FrameSpecs srcFrameSpecs,
               const FrameSpecs dstFrameSpecs,
               const ImgFrame::Type type,
               const std::array<std::array<float, 3>, 3> matrix,
               std::vector<std::array<std::array<float, 2>, 4>> srcCorners) override;
    void buildUndistort(bool enable,
                        const std::array<float, 9>& cameraMatrix,
                        const std::array<float, 9>& newCameraMatrix,
                        const std::vector<float>& distCoeffs,
                        const ImgFrame::Type type,
                        const uint32_t srcWidth,
                        const uint32_t srcHeight,
                        const uint32_t dstWidth,
                        const uint32_t dstHeight) override;

    void apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst) override;
};

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
class ColorChange {
    std::shared_ptr<spdlog::async_logger> logger;

    std::shared_ptr<ImageManipData> ccAuxFrame;
    ImgFrame::Type from;
    ImgFrame::Type to;

    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;

    bool colorConvertToRGB888i(const std::shared_ptr<ImageManipData> inputFrame,
                               std::shared_ptr<ImageManipData> outputFrame,
                               FrameSpecs srcSpecs,
                               FrameSpecs dstSpecs,
                               ImgFrame::Type from);
    bool colorConvertToBGR888p(const std::shared_ptr<ImageManipData> inputFrame,
                               std::shared_ptr<ImageManipData> outputFrame,
                               FrameSpecs srcSpecs,
                               FrameSpecs dstSpecs,
                               ImgFrame::Type from);
    bool colorConvertToRGB888p(const std::shared_ptr<ImageManipData> inputFrame,
                               std::shared_ptr<ImageManipData> outputFrame,
                               FrameSpecs srcSpecs,
                               FrameSpecs dstSpecs,
                               ImgFrame::Type from);
    bool colorConvertToBGR888i(const std::shared_ptr<ImageManipData> inputFrame,
                               std::shared_ptr<ImageManipData> outputFrame,
                               FrameSpecs srcSpecs,
                               FrameSpecs dstSpecs,
                               ImgFrame::Type from);
    bool colorConvertToNV12(const std::shared_ptr<ImageManipData> inputFrame,
                            std::shared_ptr<ImageManipData> outputFrame,
                            FrameSpecs srcSpecs,
                            FrameSpecs dstSpecs,
                            ImgFrame::Type from);
    bool colorConvertToYUV420p(const std::shared_ptr<ImageManipData> inputFrame,
                               std::shared_ptr<ImageManipData> outputFrame,
                               FrameSpecs srcSpecs,
                               FrameSpecs dstSpecs,
                               ImgFrame::Type from);
    bool colorConvertToGRAY8(const std::shared_ptr<ImageManipData> inputFrame,
                             std::shared_ptr<ImageManipData> outputFrame,
                             FrameSpecs srcSpecs,
                             FrameSpecs dstSpecs,
                             ImgFrame::Type from);
    bool colorConvert(const std::shared_ptr<ImageManipData> inputFrame,
                      std::shared_ptr<ImageManipData> outputFrame,
                      FrameSpecs srcSpecs,
                      FrameSpecs dstSpecs,
                      ImgFrame::Type from,
                      ImgFrame::Type to);

   public:
    ColorChange() = default;
    ColorChange(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    void build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo);

    void apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst);
};

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
class ImageManipOperations {
    static_assert(std::is_base_of<Warp<ImageManipBuffer, ImageManipData>, WarpBackend<ImageManipBuffer, ImageManipData>>::value,
                  "WarpBackend must be derived from Warp");
    using Container = std::vector<ManipOp>;

    static constexpr uint8_t MODE_CONVERT = 1;
    static constexpr uint8_t MODE_COLORMAP = 1 << 1;
    static constexpr uint8_t MODE_WARP = 1 << 2;

    ImageManipProperties properties;

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
    WarpBackend<ImageManipBuffer, ImageManipData> warpEngine;
    ColorChange<ImageManipBuffer, ImageManipData> clrChange;

   public:
    ImageManipOperations(ImageManipProperties props, std::shared_ptr<spdlog::async_logger> logger = nullptr) : properties(props), logger(logger) {
        preprocCc.setLogger(logger);
        warpEngine.setLogger(logger);
        clrChange.setLogger(logger);
        warpEngine.init(props);
    }

    ImageManipOperations& build(const ImageManipOpsBase<Container>& base, ImgFrame::Type outputFrameType, FrameSpecs srcFrameSpecs, ImgFrame::Type type);
    ImageManipOperations& buildUndistort(bool enable,
                                         const std::array<float, 9>& cameraMatrix,
                                         const std::array<float, 9>& newCameraMatrix,
                                         const std::vector<float>& distCoeffs,
                                         const ImgFrame::Type type,
                                         const uint32_t srcWidth,
                                         const uint32_t srcHeight,
                                         const uint32_t dstWidth,
                                         const uint32_t dstHeight);

    bool apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst);

    size_t getOutputPlaneSize(uint8_t plane = 0) const;
    size_t getOutputSize() const;
    size_t getOutputWidth() const;
    size_t getOutputHeight() const;
    size_t getOutputStride(uint8_t plane = 0) const;
    FrameSpecs getOutputFrameSpecs(ImgFrame::Type type) const;
    ImgFrame::Type getOutputFrameType() const {
        return outputFrameType;
    }
    std::vector<RotatedRect> getSrcCrops() const;
    std::array<std::array<float, 3>, 3> getMatrix() const;
    bool undistortEnabled() const {
        return warpEngine.enableUndistort;
    }

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
FrameSpecs getDstFrameSpecs(size_t width, size_t height, dai::ImgFrame::Type type);

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
                     const std::array<std::array<float, 3>, 3> matrix,
                     const std::vector<uint32_t>& background,
                     const FrameSpecs& srcImgSpecs,
                     const size_t sourceMinX,
                     const size_t sourceMinY,
                     const size_t sourceMaxX,
                     const size_t sourceMaxY);
void transformFastCV(const uint8_t* src,
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
                     uint32_t* fastCvBorder);

static inline int clampi(int val, int minv, int maxv) {
    // return val < minv ? minv : (val > maxv ? maxv : val);
    return std::clamp(val, minv, maxv);
}
static inline float clampf(float val, float minv, float maxv) {
    // return val < minv ? minv : (val > maxv ? maxv : val);
    return std::clamp(val, minv, maxv);
}

bool isTypeSupported(dai::ImgFrame::Type type);

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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToRGB888p(const std::shared_ptr<ImageManipData> inputFrame,
                                                                          std::shared_ptr<ImageManipData> outputFrame,
                                                                          FrameSpecs srcSpecs,
                                                                          FrameSpecs dstSpecs,
                                                                          dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888p;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src, src + inputSize, outputFrame->data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame->data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame->data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame->data() + dstSpecs.p1Offset);
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame->getData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getData()[p3Pos] = src[srcPos + 2];
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame->getData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getData()[p3Pos] = src[srcPos + 2];
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                    outputFrame->getData()[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame->getData()[p3Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToBGR888p(const std::shared_ptr<ImageManipData> inputFrame,
                                                                          std::shared_ptr<ImageManipData> outputFrame,
                                                                          FrameSpecs srcSpecs,
                                                                          FrameSpecs dstSpecs,
                                                                          dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888p;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame->data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame->data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame->data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src, src + inputSize, outputFrame->data());
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame->getData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getData()[p3Pos] = src[srcPos + 2];
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame->getData()[p1Pos] = src[srcPos + 0];
                    outputFrame->getData()[p2Pos] = src[srcPos + 1];
                    outputFrame->getData()[p3Pos] = src[srcPos + 2];
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                    outputFrame->getData()[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame->getData()[p3Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToRGB888i(const std::shared_ptr<ImageManipData> inputFrame,
                                                                          std::shared_ptr<ImageManipData> outputFrame,
                                                                          FrameSpecs srcSpecs,
                                                                          FrameSpecs dstSpecs,
                                                                          dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888i;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
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
                                       outputFrame->data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame->getData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getData()[dstPos + 2] = src[p3Pos];
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
                                       outputFrame->data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame->getData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i:
            std::copy(src, src + inputSize, outputFrame->data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame->getData()[dstPos + 0] = src[srcPos + 2];
                    outputFrame->getData()[dstPos + 1] = src[srcPos + 1];
                    outputFrame->getData()[dstPos + 2] = src[srcPos + 0];
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
                                                   outputFrame->data() + dstSpecs.p1Offset,
                                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                                             outputFrame->data() + dstSpecs.p1Offset,
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
                    outputFrame->getData()[pos + 0] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                    outputFrame->getData()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame->getData()[pos + 2] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToBGR888i(const std::shared_ptr<ImageManipData> inputFrame,
                                                                          std::shared_ptr<ImageManipData> outputFrame,
                                                                          FrameSpecs srcSpecs,
                                                                          FrameSpecs dstSpecs,
                                                                          dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888i;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
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
                                       outputFrame->data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame->getData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getData()[dstPos + 2] = src[p3Pos];
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
                                       outputFrame->data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame->getData()[dstPos + 0] = src[p1Pos];
                    outputFrame->getData()[dstPos + 1] = src[p2Pos];
                    outputFrame->getData()[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame->getData()[dstPos + 0] = src[srcPos + 2];
                    outputFrame->getData()[dstPos + 1] = src[srcPos + 1];
                    outputFrame->getData()[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
            std::copy(src, src + inputSize, outputFrame->data());
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
            fcvColorRGB888ToBGR888u8(
                ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            fcvColorRGB888ToBGR888u8(
                ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                    outputFrame->getData()[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    outputFrame->getData()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame->getData()[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToNV12(const std::shared_ptr<ImageManipData> inputFrame,
                                                                       std::shared_ptr<ImageManipData> outputFrame,
                                                                       FrameSpecs srcSpecs,
                                                                       FrameSpecs dstSpecs,
                                                                       dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::NV12;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame->data() + dstSpecs.p1Offset,
                                                   outputFrame->data() + dstSpecs.p2Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                                   outputFrame->data() + dstSpecs.p1Offset,
                                                   outputFrame->data() + dstSpecs.p2Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                                   outputFrame->data() + dstSpecs.p1Offset,
                                                   outputFrame->data() + dstSpecs.p2Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                                   outputFrame->data() + dstSpecs.p1Offset,
                                                   outputFrame->data() + dstSpecs.p2Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12:
            std::copy(src, src + inputSize, outputFrame->data());
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
                                outputFrame->data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelCombine2Planesu8(src + srcSpecs.p2Offset,
                                       srcSpecs.width / 2,
                                       srcSpecs.height / 2,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame->data() + dstSpecs.p2Offset,
                                       dstSpecs.p2Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat frameUV(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC2, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            cv::merge(channels, frameUV);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame->data());
            memset(outputFrame->data() + dstSpecs.p2Offset, 128, dstSpecs.p2Stride * dstSpecs.height / 2);
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToYUV420p(const std::shared_ptr<ImageManipData> inputFrame,
                                                                          std::shared_ptr<ImageManipData> outputFrame,
                                                                          FrameSpecs srcSpecs,
                                                                          FrameSpecs dstSpecs,
                                                                          dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::YUV420p;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame->data() + dstSpecs.p1Offset,
                                             outputFrame->data() + dstSpecs.p2Offset,
                                             outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                             outputFrame->data() + dstSpecs.p1Offset,
                                             outputFrame->data() + dstSpecs.p2Offset,
                                             outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                             outputFrame->data() + dstSpecs.p1Offset,
                                             outputFrame->data() + dstSpecs.p2Offset,
                                             outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                             outputFrame->data() + dstSpecs.p1Offset,
                                             outputFrame->data() + dstSpecs.p2Offset,
                                             outputFrame->data() + dstSpecs.p3Offset,
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
                    outputFrame->getData()[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame->getData()[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame->getData()[p3Pos] = static_cast<uint8_t>(V);
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
                                outputFrame->data() + dstSpecs.p1Offset,
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
                                outputFrame->data() + dstSpecs.p2Offset,
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
                                outputFrame->data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame->data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame->data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(frameUV, channels);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p:
            std::copy(src, src + inputSize, outputFrame->data());
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
bool ColorChange<ImageManipBuffer, ImageManipData>::colorConvertToGRAY8(const std::shared_ptr<ImageManipData> inputFrame,
                                                                        std::shared_ptr<ImageManipData> outputFrame,
                                                                        FrameSpecs srcSpecs,
                                                                        FrameSpecs dstSpecs,
                                                                        dai::ImgFrame::Type from) {
    // dai::ImgFrame::Type to = dai::ImgFrame::Type::GRAY8;

    auto src = inputFrame->getData().data();
    auto inputSize = inputFrame->getSize();
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
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            // Convert to grayscale
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
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
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame->data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame->data());
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
    size_t newAuxFrameSize = ALIGN_UP(srcSpecs.height, DEPTHAI_HEIGHT_ALIGNMENT) * ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
    if(!ccAuxFrame || ccAuxFrame->size() < newAuxFrameSize) ccAuxFrame = std::make_shared<ImageManipData>(newAuxFrameSize);
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void ColorChange<ImageManipBuffer, ImageManipData>::apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst) {
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
        std::copy(src->data(), src->data() + (src->size() <= dst->size() ? src->size() : dst->size()), dst->data());
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

inline bool floatEq(float a, float b) {
    return fabsf(a - b) <= 1e-6f;
}

inline bool isSingleChannelu8(const std::shared_ptr<dai::ImgFrame> img) {
    return img->getType() == dai::ImgFrame::Type::GRAY8 || img->getType() == dai::ImgFrame::Type::RAW8;
}
inline bool isSingleChannelu8(const dai::ImgFrame::Type type) {
    return type == dai::ImgFrame::Type::GRAY8 || type == dai::ImgFrame::Type::RAW8;
}
inline bool isSingleChannel(const dai::ImgFrame::Type type) {
    return type == dai::ImgFrame::Type::GRAY8 || type == dai::ImgFrame::Type::RAW8 || type == dai::ImgFrame::Type::RAW16 || type == dai::ImgFrame::Type::GRAYF16
           || type == dai::ImgFrame::Type::RAW32;
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
    configSS << "| o=" << ops.outputWidth << "x" << ops.outputHeight << " c=" << ops.center << " rm=" << (int)ops.resizeMode << " b=" << (int)ops.background
             << " bc=" << ops.backgroundR << "," << ops.backgroundG << "," << ops.backgroundB << " c=" << (int)ops.colormap << " u=" << (int)ops.undistort;
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

void getOutputSizeFromCorners(const std::array<std::array<float, 2>, 4>& corners,
                              const bool center,
                              const std::array<std::array<float, 3>, 3> transformInv,
                              const uint32_t srcWidth,
                              const uint32_t srcHeight,
                              uint32_t& outputWidth,
                              uint32_t& outputHeight);

template <typename C>
std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>, std::vector<std::array<std::array<float, 2>, 4>>> getFullTransform(
    dai::ImageManipOpsBase<C>& base, size_t inputWidth, size_t inputHeight, dai::ImgFrame::Type type, dai::ImgFrame::Type outputFrameType, C& outputOps) {
    using namespace dai;
    using namespace dai::impl;

    outputOps.clear();

    auto operations = base.getOperations();

    auto [matrix, imageCorners, srcCorners] = getTransform(operations, inputWidth, inputHeight, base.outputWidth, base.outputHeight);

    getOutputSizeFromCorners(imageCorners, base.center, getInverse(matrix), inputWidth, inputHeight, base.outputWidth, base.outputHeight);

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

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>& ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::build(
    const ImageManipOpsBase<Container>& newBase, ImgFrame::Type outType, FrameSpecs srcFrameSpecs, ImgFrame::Type inFrameType) {
    const auto newCfgStr = newBase.str();
    if(outType == ImgFrame::Type::NONE) {
        if(base.colormap != Colormap::NONE)
            outType = VALID_TYPE_COLOR;
        else
            outType = inFrameType;
    }
    if(newCfgStr == prevConfig && outType == outputFrameType && srcFrameSpecs.width == srcSpecs.width && srcFrameSpecs.height == srcSpecs.height
       && inFrameType == inType)
        return *this;
    prevConfig = newCfgStr;
    outputOps.clear();

    if(srcFrameSpecs.width <= 1 || srcFrameSpecs.height <= 1) {
        throw std::runtime_error("Input image is one dimensional");
    }

    if(newBase.hasWarp(srcFrameSpecs.width, srcFrameSpecs.height)) mode = mode | MODE_WARP;
    if(newBase.colormap != Colormap::NONE && isSingleChannelu8(inFrameType)) mode = mode | MODE_COLORMAP;
    if(outType != inFrameType) mode = mode | MODE_CONVERT;

    assert(inFrameType != ImgFrame::Type::NONE);
    base = newBase;
    outputFrameType = outType;
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
        if(!isTypeSupported(inType)) {
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
    size_t newConvertedSize = getAlignedOutputFrameSize(type, inputWidth, inputHeight);
    size_t newColormapSize = getAlignedOutputFrameSize(type, base.outputWidth, base.outputHeight);
    size_t newWarpedSize =
        getAlignedOutputFrameSize(isSingleChannelu8(type) && base.colormap != Colormap::NONE ? VALID_TYPE_COLOR : type, base.outputWidth, base.outputHeight);

    if(!convertedFrame || convertedFrame->size() < newConvertedSize) convertedFrame = std::make_shared<ImageManipData>(newConvertedSize);
    if(!colormapFrame || colormapFrame->size() < newColormapSize) colormapFrame = std::make_shared<ImageManipData>(newColormapSize);
    if(!warpedFrame || warpedFrame->size() < newWarpedSize) warpedFrame = std::make_shared<ImageManipData>(newWarpedSize);

    return *this;
}  // namespace impl

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>& ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::buildUndistort(
    bool enable,
    const std::array<float, 9>& cameraMatrix,
    const std::array<float, 9>& newCameraMatrix,
    const std::vector<float>& distCoeffs,
    const ImgFrame::Type type,
    const uint32_t srcWidth,
    const uint32_t srcHeight,
    const uint32_t dstWidth,
    const uint32_t dstHeight) {
    warpEngine.buildUndistort(enable, cameraMatrix, newCameraMatrix, distCoeffs, type, srcWidth, srcHeight, dstWidth, dstHeight);
    return *this;
}

size_t getFrameSize(const ImgFrame::Type type, const FrameSpecs& specs);

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
bool ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::apply(const std::shared_ptr<ImageManipData> src,
                                                                                std::shared_ptr<ImageManipData> dst) {
    size_t requiredSize = getFrameSize(inType, srcSpecs);
    if(src->size() < requiredSize) throw std::runtime_error("ImageManip not built for the source image specs. Consider rebuilding with the new configuration.");
    if(mode == 0) {
        std::copy(src->getData().begin(), src->getData().end(), dst->getData().begin());
        return true;
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(convertInput || mode == MODE_CONVERT) preprocCc.apply(src, mode == MODE_CONVERT ? dst : convertedFrame);
    if(mode != MODE_CONVERT) {
        if(mode & MODE_WARP) {
            warpEngine.apply(convertInput ? convertedFrame : src,
                             base.colormap != Colormap::NONE ? colormapFrame : (type == outputFrameType ? dst : warpedFrame));
        }
        if(mode & MODE_COLORMAP) {
            uint8_t* colormapDst = outputFrameType == VALID_TYPE_COLOR ? dst->data() : warpedFrame->data();
            size_t colormapDstStride = outputFrameType == VALID_TYPE_COLOR ? getOutputStride() : ALIGN_UP(base.outputWidth, DEPTHAI_STRIDE_ALIGNMENT);
            uint8_t* colormapSrc = mode & MODE_WARP ? colormapFrame->data() : (convertInput ? convertedFrame->data() : src->getData().data());
            size_t colormapSrcStride = !(mode & MODE_WARP) && !convertInput ? srcSpecs.p1Stride : ALIGN_UP(base.outputWidth, DEPTHAI_STRIDE_ALIGNMENT);
            cv::Mat gray(base.outputWidth, base.outputHeight, CV_8UC1, colormapSrc, colormapSrcStride);
            cv::Mat color(base.outputWidth, base.outputHeight, CV_8UC3, colormapDst, colormapDstStride);
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
            clrChange.apply(warpedFrame, dst);
        }
    }
    return true;  // TODO(asahtik): Handle failed transformation
#else
    return false;
#endif
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputWidth() const {
    return base.outputWidth;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputHeight() const {
    return base.outputHeight;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputStride(uint8_t plane) const {
    if(mode == 0) return plane == 0 ? srcSpecs.p1Stride : (plane == 1 ? srcSpecs.p2Stride : (plane == 2 ? srcSpecs.p3Stride : 0));
    auto specs = getOutputFrameSpecs(outputFrameType);
    if(plane == 0)
        return specs.p1Stride;
    else if(plane == 1)
        return specs.p2Stride;
    else if(plane == 2)
        return specs.p3Stride;
    else
        return 0;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputPlaneSize(uint8_t plane) const {
    if(mode == 0) return 0;
    size_t size = 0;
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            size = getOutputStride() * getOutputHeight();  // Do not do stride for RGB/BGRi/p
            break;
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
            size = getOutputStride() * ALIGN_UP(getOutputHeight(), DEPTHAI_HEIGHT_ALIGNMENT);
            break;
        case ImgFrame::Type::NV12:
            if(plane == 0) {
                size = getOutputStride(0) * ALIGN_UP(getOutputHeight(), DEPTHAI_HEIGHT_ALIGNMENT);
            } else if(plane == 1) {
                size = ALIGN_UP(getOutputStride(1) * ALIGN_UP(getOutputHeight() / 2, DEPTHAI_HEIGHT_ALIGNMENT / 2), DEPTHAI_PLANE_ALIGNMENT);
            }
            break;
        case ImgFrame::Type::YUV420p:
            if(plane == 0) {
                size = getOutputStride(0) * ALIGN_UP(getOutputHeight(), DEPTHAI_HEIGHT_ALIGNMENT);
            } else if(plane == 1 || plane == 2) {
                size = ALIGN_UP(getOutputStride(plane) * ALIGN_UP(getOutputHeight() / 2, DEPTHAI_HEIGHT_ALIGNMENT / 2), DEPTHAI_PLANE_ALIGNMENT);
            }
            break;
        case ImgFrame::Type::RAW16:
            size = getOutputStride() * getOutputHeight();  // Do not do stride for RGB/BGRi/p
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
            throw std::runtime_error("Output frame type not supported");
    }
    if(size == 0) throw std::runtime_error("Output size is 0 for plane " + std::to_string(plane));
    return size;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
size_t ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputSize() const {
    if(mode == 0) return 0;
    size_t size = 0;
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            size = getOutputPlaneSize(0) * 3;
            break;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::RAW16:
            size = getOutputPlaneSize(0);
            break;
        case ImgFrame::Type::YUV420p:
            size = getOutputPlaneSize(2);
            // Fallthrough
        case ImgFrame::Type::NV12:
            size += getOutputPlaneSize(0) + getOutputPlaneSize(1);
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
            throw std::runtime_error("Output frame type not supported");
    }
    if(size == 0) throw std::runtime_error("Output size is 0");
    return size;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
FrameSpecs ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getOutputFrameSpecs(ImgFrame::Type type) const {
    if(mode == 0)
        return srcSpecs;
    else
        return getDstFrameSpecs(base.outputWidth, base.outputHeight, type);
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
std::vector<RotatedRect> ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getSrcCrops() const {
    std::vector<RotatedRect> crops;
    for(const auto& corners : srcCorners) {
        auto rect = getRotatedRectFromPoints({corners[0], corners[1], corners[2], corners[3]});
        crops.push_back(rect);
    }
    return crops;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
std::array<std::array<float, 3>, 3> ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::getMatrix() const {
    return matrix;
}

template <template <typename T> typename ImageManipBuffer,
          typename ImageManipData,
          template <template <typename T> typename Buf, typename Dat> typename WarpBackend>
std::string ImageManipOperations<ImageManipBuffer, ImageManipData, WarpBackend>::toString() const {
    std::stringstream cStr;
    cStr << getConfigString(base);
    if(outputOps.size() > 0) {
        cStr << " | ";
        for(auto i = 0U; i < outputOps.size(); ++i) {
            cStr << std::visit([](auto&& op) { return getOpStr(op); }, outputOps[i].op);
            if(i != outputOps.size() - 1) cStr << " ";
        }
    }
    return cStr.str();
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void WarpH<ImageManipBuffer, ImageManipData>::build(const FrameSpecs srcFrameSpecs,
                                                    const FrameSpecs dstFrameSpecs,
                                                    const ImgFrame::Type type,
                                                    const std::array<std::array<float, 3>, 3> matrix,
                                                    std::vector<std::array<std::array<float, 2>, 4>> srcCorners) {
    this->matrix = matrix;
    this->type = type;
    this->srcSpecs = srcFrameSpecs;
    this->dstSpecs = dstFrameSpecs;

    if(!fastCvBorder || fastCvBorder->size() < this->dstSpecs.height * 2)
        fastCvBorder = std::make_shared<ImageManipBuffer<uint32_t>>(this->dstSpecs.height * 2);

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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void WarpH<ImageManipBuffer, ImageManipData>::buildUndistort(bool enable,
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
            if(!auxFrame || auxFrame->size() < frameSize) {
                auxFrame = std::make_shared<ImageManipData>(frameSize);
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void WarpH<ImageManipBuffer, ImageManipData>::transform(const std::shared_ptr<ImageManipData> src,
                                                        std::shared_ptr<ImageManipData> dst,
                                                        const size_t srcWidth,
                                                        const size_t srcHeight,
                                                        const size_t srcStride,
                                                        const size_t dstWidth,
                                                        const size_t dstHeight,
                                                        const size_t dstStride,
                                                        const uint16_t numChannels,
                                                        const uint16_t bpp,
                                                        const std::array<std::array<float, 3>, 3> matrix,
                                                        const std::vector<uint32_t>& background) {
    if(1) {
#ifdef DEPTHAI_IMAGEMANIPV2_OPENCV
        transformOpenCV(src->data(),
                        dst->data(),
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
    } else {
#ifdef DEPTHAI_IMAGEMANIPV2_FASTCV
        transformFastCV(src->data()),
                        dst->data(),
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
                        this->sourceMaxY,
                        fastCvBorder->data());
#else
        throw std::runtime_error("FastCV backend not available");
#endif
    }

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

void printSpecs(spdlog::async_logger& logger, FrameSpecs specs);

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
void WarpH<ImageManipBuffer, ImageManipData>::apply(const std::shared_ptr<ImageManipData> src, std::shared_ptr<ImageManipData> dst) {
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
                cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC3, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                cv::Mat dstCv(
                    undistortSpecs.height, undistortSpecs.width, CV_8UC3, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
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
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p2Offset)->data(), this->srcSpecs.p2Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p2Offset)->data(), undistortSpecs.p2Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p3Offset)->data(), this->srcSpecs.p3Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p3Offset)->data(), undistortSpecs.p3Stride);
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
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(
                        this->srcSpecs.height / 2, this->srcSpecs.width / 2, CV_8UC1, src->offset(this->srcSpecs.p2Offset)->data(), this->srcSpecs.p2Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p2Offset)->data(),
                                  undistortSpecs.p2Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(
                        this->srcSpecs.height / 2, this->srcSpecs.width / 2, CV_8UC1, src->offset(this->srcSpecs.p3Offset)->data(), this->srcSpecs.p3Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC1,
                                  undistortDst->offset(undistortSpecs.p3Offset)->data(),
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
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
                    this->undistortImpl->undistort(srcCv, dstCv);
                }
                {
                    cv::Mat srcCv(
                        this->srcSpecs.height / 2, this->srcSpecs.width / 2, CV_8UC2, src->offset(this->srcSpecs.p2Offset)->data(), this->srcSpecs.p2Stride);
                    cv::Mat dstCv(undistortSpecs.height / 2,
                                  undistortSpecs.width / 2,
                                  CV_8UC2,
                                  undistortDst->offset(undistortSpecs.p2Offset)->data(),
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
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_8UC1, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_8UC1, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
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
                    cv::Mat srcCv(this->srcSpecs.height, this->srcSpecs.width, CV_16UC1, src->offset(this->srcSpecs.p1Offset)->data(), this->srcSpecs.p1Stride);
                    cv::Mat dstCv(
                        undistortSpecs.height, undistortSpecs.width, CV_16UC1, undistortDst->offset(undistortSpecs.p1Offset)->data(), undistortSpecs.p1Stride);
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

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
bool Warp<ImageManipBuffer, ImageManipData>::isIdentityWarp() const {
    return (matrix[0][0] == 1.0f && matrix[0][1] == 0.0f && matrix[0][2] == 0.0f && matrix[1][0] == 0.0f && matrix[1][1] == 1.0f && matrix[1][2] == 0.0f
            && matrix[2][0] == 0.0f && matrix[2][1] == 0.0f && matrix[2][2] == 1.0f)
           && (srcSpecs.width == dstSpecs.width && srcSpecs.height == dstSpecs.height);
}

template <template <typename T> typename ImageManipBuffer, typename ImageManipData>
Warp<ImageManipBuffer, ImageManipData>& Warp<ImageManipBuffer, ImageManipData>::setBackgroundColor(const uint32_t r, const uint32_t g, const uint32_t b) {
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

}  // namespace impl
}  // namespace dai

#undef _RESTRICT

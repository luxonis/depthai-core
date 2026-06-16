#pragma once
#include <string>
#define _USE_MATH_DEFINES

#include <spdlog/async_logger.h>
#include <stdint.h>

#include <cmath>
#include <sstream>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "depthai/utility/matrixOps.hpp"

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
template <typename N>
void loop(N& node,
          const ImageManipConfig& initialConfig,
          std::shared_ptr<spdlog::async_logger> logger,
          const std::function<size_t(const ImageManipConfig&, const ImgFrame&)>& build,
          const std::function<std::shared_ptr<ImgFrame>(size_t)>& getFrame,
          const std::function<bool(const std::shared_ptr<OffsetMemory>&, std::shared_ptr<OffsetMemory>)>& apply,
          const std::function<void(const ImgFrame&, ImgFrame&)>& setFrame) {
    using namespace std::chrono;
    auto config = initialConfig;

    std::shared_ptr<ImgFrame> inImage;

    while(node.mainLoop()) {
        std::shared_ptr<ImageManipConfig> pConfig;
        bool hasConfig = false;
        bool needsImage = true;
        bool skipImage = false;
        {
            auto blockEvent = node.inputBlockEvent();

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
        }

        auto startP = std::chrono::steady_clock::now();

        auto t1 = steady_clock::now();
        auto outputSize = build(config, *inImage);
        auto t2 = steady_clock::now();

        // Check the output image size requirements, and check whether pool has the size required
        if(outputSize == 0) {
            node.out.send(inImage);
        } else if((long)outputSize <= (long)node.properties.outputFrameSize) {
            auto outImage = getFrame(node.properties.outputFrameSize);

            bool success = true;
            {
                auto t3 = steady_clock::now();
                success = apply(ConvertedOffsetMemory::convert(inImage->data), ConvertedOffsetMemory::convert(outImage->data));
                auto t4 = steady_clock::now();

                setFrame(*inImage, *outImage);

                logger->trace("Build time: {}us, Process time: {}us, Total time: {}us, image manip id: {}",
                              duration_cast<microseconds>(t2 - t1).count(),
                              duration_cast<microseconds>(t4 - t3).count(),
                              duration_cast<microseconds>(t4 - t1).count(),
                              node.id);
            }
            if(!success) {
                logger->error("Processing failed, potentially unsupported config");
            }
            {
                auto blockEvent = node.outputBlockEvent();

                node.out.send(outImage);
            }
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

class _ImageManipMemory : public OffsetMemory {
    std::shared_ptr<std::vector<uint8_t>> _data;
    span<uint8_t> _span;
    size_t _offset = 0;

    uint8_t* data() {
        return _span.data() + _offset;
    }
    const uint8_t* data() const {
        return _span.data() + _offset;
    }
    size_t size() const {
        return _span.size() - _offset;
    }

   public:
    _ImageManipMemory() = default;
    _ImageManipMemory(size_t size) {
        _data = std::make_shared<std::vector<uint8_t>>(size);
        _span = span(*_data);
    }
    _ImageManipMemory(span<uint8_t> data) : _span(data) {}
    span<uint8_t> getData() override {
        return _span;
    }
    span<const uint8_t> getData() const override {
        return _span;
    }
    span<uint8_t> getOffsetData() override {
        return span(_span.data() + _offset, _span.data() + _offset + size());
    }
    span<const uint8_t> getOffsetData() const override {
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
    void setOffset(size_t offset) override {
        _offset = std::min(_offset + offset, _span.size());
    }
    void shallowCopyFrom(_ImageManipMemory& other) {
        _data = other._data;
        _span = other._span;
        _offset = other._offset;
    }
    std::shared_ptr<OffsetMemory> offset(size_t offset) override {
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

    bool validMatrix(const std::array<float, 9>& matrix) const;
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

    virtual void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) = 0;

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    Warp& setBackgroundColor(uint32_t r, uint32_t g, uint32_t b);
};

class WarpH : public Warp {
    std::shared_ptr<_ImageManipBuffer<uint32_t>> fastCvBorder;
    std::shared_ptr<_ImageManipMemory> auxFrame;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<UndistortOpenCvImpl> undistortImpl;
#else
    std::unique_ptr<uint32_t> dummyUndistortImpl;
#endif

    void transform(const std::shared_ptr<OffsetMemory>& srcData,
                   const std::shared_ptr<OffsetMemory>& dstData,
                   const size_t srcWidth,
                   const size_t srcHeight,
                   const size_t srcStride,
                   const size_t dstWidth,
                   const size_t dstHeight,
                   const size_t dstStride,
                   const uint16_t numChannels,
                   const uint16_t bpp,
                   const std::array<std::array<float, 3>, 3>& matrix,
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

    void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) override;
};

class ColorChange {
   protected:
    std::shared_ptr<spdlog::async_logger> logger;

    ImgFrame::Type from;
    ImgFrame::Type to;

    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;

   public:
    ColorChange() = default;
    ColorChange(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}
    virtual ~ColorChange() = default;

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    virtual void build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo) = 0;

    virtual void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) = 0;
};

class ColorChangeH : public ColorChange {
    std::shared_ptr<_ImageManipMemory> ccAuxFrame;

   public:
    void build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo);

    void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst);
};

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
class ImageManipOperations {
    static_assert(std::is_base_of<OffsetMemory, ImageManipData>::value, "ImageManipData must be derived from OffsetMemory");
    static_assert(std::is_base_of<Warp, WarpBackend>::value, "WarpBackend must be derived from Warp");
    static_assert(std::is_base_of<ColorChange, ColorChangeBackend>::value, "ColorChangeBackend must be derived from ColorChange");
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

    ColorChangeBackend preprocCc;
    WarpBackend warpEngine;
    ColorChangeBackend clrChange;

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

    bool apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst);

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

struct ColorChangeArgs {
    const std::shared_ptr<OffsetMemory>& inputFrame;
    std::shared_ptr<OffsetMemory> outputFrame;
    FrameSpecs srcSpecs;
    FrameSpecs dstSpecs;
    ImgFrame::Type from;
    std::shared_ptr<OffsetMemory> auxFrame;
};
FrameSpecs getSrcFrameSpecs(dai::ImgFrame::Specs srcSpecs);
FrameSpecs getCcDstFrameSpecs(const FrameSpecs& srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to);
FrameSpecs getDstFrameSpecs(size_t width, size_t height, dai::ImgFrame::Type type);
size_t getAlignedOutputFrameSize(ImgFrame::Type type, size_t width, size_t height);
void printSpecs(spdlog::async_logger& logger, FrameSpecs specs);
size_t getFrameSize(const ImgFrame::Type type, const FrameSpecs& specs);
bool getFrameTypeInfo(dai::ImgFrame::Type outFrameType, int& outNumPlanes, float& outBpp);

bool isTypeSupported(dai::ImgFrame::Type type);

bool colorConvertToRGB888p(const ColorChangeArgs& args);
bool colorConvertToBGR888p(const ColorChangeArgs& args);
bool colorConvertToRGB888i(const ColorChangeArgs& args);
bool colorConvertToBGR888i(const ColorChangeArgs& args);
bool colorConvertToNV12(const ColorChangeArgs& args);
bool colorConvertToYUV420p(const ColorChangeArgs& args);
bool colorConvertToGRAY8(const ColorChangeArgs& args);

std::tuple<float, float, float, float> getOuterRect(const std::vector<std::array<float, 2>>& points);
dai::RotatedRect getOuterRotatedRect(const std::vector<std::array<float, 2>>& points);
std::array<std::array<float, 3>, 3> getResizeMat(Resize o, float width, float height, uint32_t outputWidth, uint32_t outputHeight);
void getOutputSizeFromCorners(const std::array<std::array<float, 2>, 4>& corners,
                              const bool center,
                              const std::array<std::array<float, 3>, 3>& transformInv,
                              const uint32_t srcWidth,
                              const uint32_t srcHeight,
                              uint32_t& outputWidth,
                              uint32_t& outputHeight);

void getTransformImpl(const ManipOp& op,
                      std::array<std::array<float, 3>, 3>& transform,
                      std::array<std::array<float, 2>, 4>& imageCorners,
                      std::vector<std::array<std::array<float, 2>, 4>>& srcCorners,
                      uint32_t& outputWidth,
                      uint32_t& outputHeight);

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

template <typename C>
std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>, std::vector<std::array<std::array<float, 2>, 4>>> getFullTransform(
    dai::ImageManipOpsBase<C>& base, size_t inputWidth, size_t inputHeight, dai::ImgFrame::Type type, dai::ImgFrame::Type outputFrameType, C& outputOps) {
    using namespace dai;
    using namespace dai::impl;

    outputOps.clear();

    auto operations = base.getOperations();

    auto [matrix, imageCorners, srcCorners] = getTransform(operations, inputWidth, inputHeight, base.outputWidth, base.outputHeight);

    getOutputSizeFromCorners(imageCorners, base.center, matrix::getMatrixInverse(matrix), inputWidth, inputHeight, base.outputWidth, base.outputHeight);

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

    auto matrixInv = matrix::getMatrixInverse(matrix);

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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>& ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::build(
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
        this->matrixInv = matrix::getMatrixInverse(matrix);
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

    if(!convertedFrame || convertedFrame->getOffsetSize() < newConvertedSize) convertedFrame = std::make_shared<ImageManipData>(newConvertedSize);
    if(!colormapFrame || colormapFrame->getOffsetSize() < newColormapSize) colormapFrame = std::make_shared<ImageManipData>(newColormapSize);
    if(!warpedFrame || warpedFrame->getOffsetSize() < newWarpedSize) warpedFrame = std::make_shared<ImageManipData>(newWarpedSize);

    return *this;
}  // namespace impl

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>& ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::buildUndistort(
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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
bool ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
    size_t requiredSize = getFrameSize(inType, srcSpecs);
    if(src->getOffsetSize() < requiredSize)
        throw std::runtime_error("ImageManip not built for the source image specs. Consider rebuilding with the new configuration.");
    if(mode == 0) {
        std::copy(src->getOffsetData().begin(), src->getOffsetData().end(), dst->getOffsetData().begin());
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
            uint8_t* colormapDst = outputFrameType == VALID_TYPE_COLOR ? dst->getOffsetData().data() : warpedFrame->getOffsetData().data();
            size_t colormapDstStride = outputFrameType == VALID_TYPE_COLOR ? getOutputStride() : ALIGN_UP(base.outputWidth, DEPTHAI_STRIDE_ALIGNMENT);
            uint8_t* colormapSrc = mode & MODE_WARP ? colormapFrame->getOffsetData().data()
                                                    : (convertInput ? convertedFrame->getOffsetData().data() : src->getOffsetData().data());
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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
size_t ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputWidth() const {
    return base.outputWidth;
}

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
size_t ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputHeight() const {
    return base.outputHeight;
}

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
size_t ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputStride(uint8_t plane) const {
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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
size_t ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputPlaneSize(uint8_t plane) const {
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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
size_t ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputSize() const {
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

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
FrameSpecs ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getOutputFrameSpecs(ImgFrame::Type type) const {
    if(mode == 0)
        return srcSpecs;
    else
        return getDstFrameSpecs(base.outputWidth, base.outputHeight, type);
}

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
std::vector<RotatedRect> ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getSrcCrops() const {
    std::vector<RotatedRect> crops;
    for(const auto& corners : srcCorners) {
        auto rect = getOuterRotatedRect({corners[0], corners[1], corners[2], corners[3]});
        crops.push_back(rect);
    }
    return crops;
}

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
std::array<std::array<float, 3>, 3> ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::getMatrix() const {
    return matrix;
}

template <typename ImageManipData, typename ColorChangeBackend, typename WarpBackend>
std::string ImageManipOperations<ImageManipData, ColorChangeBackend, WarpBackend>::toString() const {
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

}  // namespace impl
}  // namespace dai

#undef _RESTRICT

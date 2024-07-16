#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfigV2.hpp>

// shared
#include <depthai/properties/ImageManipPropertiesV2.hpp>

#ifdef TARGET_DEVICE_RVC4
    #include "utilities/EvaDataMemory.hpp"
#endif

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManipV2 : public DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2> {
   private:
    bool runOnHostVar = false;

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "ImageManipV2";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    ImageManipV2() = default;
    ImageManipV2(std::unique_ptr<Properties> props);

    std::shared_ptr<ImageManipV2> build() {
        return std::static_pointer_cast<ImageManipV2>(shared_from_this());
    }
    /**
     * Initial config to use when manipulating frames
     */
    ImageManipConfigV2 initialConfig;

    /**
     * Input ImageManipConfigV2 message with ability to modify parameters in runtime
     */
    Input inputConfig{*this, {.name = "inputConfig", .types = {{DatatypeEnum::ImageManipConfigV2, true}}, .waitForMessage = false}};

    /**
     * Input image to be modified
     */
    Input inputImage{*this, {.name = "inputImage", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify maximum size of output image.
     * @param maxFrameSize Maximum frame size in bytes
     */
    void setMaxOutputFrameSize(int maxFrameSize);

    /**
     * Specify whether to run on host or device
     * @param runOnHost Run node on host
     */
    ImageManipV2& setRunOnHost(bool runOnHost = true);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;
};

}  // namespace node

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

#ifdef TARGET_DEVICE_RVC4
using ImageManipMemory = dai::EvaDataMemory;
template <typename T>
using ImageManipBuffer = dai::EvaBuffer<T>;
#else
using ImageManipMemory = _ImageManipMemory;
template <typename T>
using ImageManipBuffer = _ImageManipBuffer<T>;
#endif

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

class WarpEngine {
    std::shared_ptr<spdlog::async_logger> logger;

    std::array<std::array<float, 3>, 3> matrix;

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

   public:
    WarpEngine() = default;
    WarpEngine(std::shared_ptr<spdlog::async_logger> logger) : logger(logger) {}

    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = logger;
    }

    void build(const FrameSpecs srcFrameSpecs,
               const FrameSpecs dstFrameSpecs,
               const ImgFrame::Type type,
               const std::array<std::array<float, 3>, 3> matrix,
               std::vector<std::array<std::array<float, 2>, 4>> srcCorners);

    void apply(const span<const uint8_t> src, span<uint8_t> dst);
};

class ColorChange {
    std::shared_ptr<spdlog::async_logger> logger;

    std::shared_ptr<ImageManipMemory> ccAuxFrame;
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

class ImageManipOperations {
    static constexpr uint8_t MODE_CONVERT = 1;
    static constexpr uint8_t MODE_COLORMAP = 1 << 1;
    static constexpr uint8_t MODE_WARP = 1 << 2;

    uint8_t mode = 0;
    std::string prevConfig;

    std::array<std::array<float, 3>, 3> matrix{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 3>, 3> matrixInv{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    ImageManipOpsBase base;
    ImgFrame::Type outputFrameType;
    ImgFrame::Type type;
    ImgFrame::Type inType;

    bool convertInput = false;
    bool convertOnly = false;

    std::shared_ptr<ImageManipMemory> colormapFrame;
    std::shared_ptr<ImageManipMemory> convertedFrame;
    std::shared_ptr<ImageManipMemory> warpedFrame;

    std::shared_ptr<spdlog::async_logger> logger;

    FrameSpecs srcSpecs;

    ColorChange preprocCc;
    WarpEngine warpEngine;
    ColorChange clrChange;

   public:
    ImageManipOperations(std::shared_ptr<spdlog::async_logger> logger = nullptr) : logger(logger) {
        preprocCc.setLogger(logger);
        warpEngine.setLogger(logger);
        clrChange.setLogger(logger);
    }

    void init();

    ImageManipOperations& build(const ImageManipOpsBase& base, ImgFrame::Type outputFrameType, FrameSpecs srcFrameSpecs, ImgFrame::Type type);

    bool apply(const std::shared_ptr<ImgFrame> src, span<uint8_t> dst);

    size_t getOutputSize() const;
    size_t getOutputWidth() const;
    size_t getOutputHeight() const;
    size_t getOutputStride(uint8_t plane = 0) const;
    FrameSpecs getOutputFrameSpecs(ImgFrame::Type type) const;
};

}  // namespace impl
}  // namespace dai

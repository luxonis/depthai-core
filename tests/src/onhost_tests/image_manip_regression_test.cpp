// Regression tests for the ImageManip implementation shared by the host node and the RVC4 firmware.
#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ImageManipImpl.hpp"

using dai::ImageManipConfig;
using dai::ImgFrame;
using namespace dai::impl;

namespace {

using Manip = ImageManipOperations<_ImageManipMemory, ColorChangeH, WarpH>;
using PixelFn = std::function<uint8_t(uint32_t x, uint32_t y, uint32_t c)>;

struct Image {
    ImgFrame::Type type;
    FrameSpecs specs;
    std::shared_ptr<_ImageManipMemory> data;

    uint8_t* bytes() const {
        return data->getOffsetData().data();
    }
};

/// A pattern without any symmetry, so that misplaced pixels always show
uint8_t pattern(uint32_t x, uint32_t y, uint32_t c) {
    return static_cast<uint8_t>((x * 7 + y * 13 + c * 61 + (x * y) % 17) & 0xFF);
}

/// Packed RGB888i / BGR888i / GRAY8 / RAW8 image. A stride larger than the row size leaves padding filled with 0xEE.
Image makePacked(ImgFrame::Type type, uint32_t width, uint32_t height, const PixelFn& value = pattern, uint32_t stride = 0) {
    const uint32_t channels = type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i ? 3 : 1;
    if(stride == 0) stride = width * channels;
    Image img{type, FrameSpecs{width, height, 0, 0, 0, stride, 0, 0}, std::make_shared<_ImageManipMemory>(stride * height)};
    std::fill(img.bytes(), img.bytes() + stride * height, 0xEE);
    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) {
            for(uint32_t c = 0; c < channels; ++c) img.bytes()[y * stride + x * channels + c] = value(x, y, c);
        }
    }
    return img;
}

/// Builds the operations for cfg and processes `in`. Returns nullopt if processing failed.
std::optional<Image> run(Manip& manip, const ImageManipConfig& cfg, const Image& in) {
    manip.build(cfg.base, cfg.outputFrameType, in.specs, in.type);
    const size_t outputSize = manip.getOutputSize();
    if(outputSize == 0) return in;  // Passthrough: the node forwards the input frame
    auto out = std::make_shared<_ImageManipMemory>(outputSize);
    if(!manip.apply(in.data, out)) return std::nullopt;
    const auto type = manip.getOutputFrameType();
    return Image{type, manip.getOutputFrameSpecs(type), out};
}

std::optional<Image> run(const ImageManipConfig& cfg, const Image& in) {
    Manip manip({});
    return run(manip, cfg, in);
}

}  // namespace

TEST_CASE("ImageManip converts single channel images to full range YUV", "[ImageManip][ColorConversion]") {
    const auto target = GENERATE(ImgFrame::Type::NV12, ImgFrame::Type::YUV420p);
    // 80: padded input rows, so that the conversion has to respect the strides of input and output
    const auto inputStride = GENERATE(64u, 80u);
    CAPTURE(target, inputStride);
    const auto gray = makePacked(ImgFrame::Type::GRAY8, 64, 48, pattern, inputStride);

    ImageManipConfig convert;
    convert.setFrameType(target);
    auto out = run(convert, gray);
    REQUIRE(out);
    REQUIRE(out->type == target);
    const auto& specs = out->specs;
    REQUIRE(specs.width == 64);
    REQUIRE(specs.height == 48);
    const uint8_t* bytes = out->bytes();

    // The input is the luma plane as it is (full range), the chroma is neutral
    size_t lumaMismatches = 0;
    for(uint32_t y = 0; y < specs.height; ++y) {
        for(uint32_t x = 0; x < specs.width; ++x) lumaMismatches += bytes[specs.p1Offset + y * specs.p1Stride + x] != pattern(x, y, 0);
    }
    CHECK(lumaMismatches == 0);
    size_t chromaMismatches = 0;
    for(uint32_t y = 0; y < specs.height / 2; ++y) {
        if(target == ImgFrame::Type::NV12) {
            for(uint32_t x = 0; x < specs.width; ++x) chromaMismatches += bytes[specs.p2Offset + y * specs.p2Stride + x] != 128;
        } else {
            for(uint32_t x = 0; x < specs.width / 2; ++x) {
                chromaMismatches += bytes[specs.p2Offset + y * specs.p2Stride + x] != 128;
                chromaMismatches += bytes[specs.p3Offset + y * specs.p3Stride + x] != 128;
            }
        }
    }
    CHECK(chromaMismatches == 0);
}

TEST_CASE("ImageManip reports unsupported conversions instead of producing a frame", "[ImageManip][ColorConversion]") {
    const auto input = makePacked(ImgFrame::Type::RGB888i, 64, 48);
    ImageManipConfig convert;
    convert.setFrameType(ImgFrame::Type::RAW16);
    CHECK_FALSE(run(convert, input));

    ImageManipConfig cropAndConvert;
    cropAndConvert.addCrop(8, 4, 32, 24);
    cropAndConvert.setFrameType(ImgFrame::Type::RAW16);
    CHECK_FALSE(run(cropAndConvert, input));
}

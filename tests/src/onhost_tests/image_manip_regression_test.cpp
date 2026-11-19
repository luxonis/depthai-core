// Regression tests for the ImageManip implementation shared by the host node and the RVC4 firmware.
#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

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
    /// Packed (interleaved / single channel) pixel
    uint8_t at(uint32_t x, uint32_t y, uint32_t c = 0) const {
        const uint32_t channels = type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i ? 3 : 1;
        return bytes()[specs.p1Offset + y * specs.p1Stride + x * channels + c];
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

/// Number of pixels (of a packed w x h region) where out(x, y) != expected(x, y)
size_t countMismatches(const Image& out, uint32_t width, uint32_t height, uint32_t channels, const PixelFn& expected) {
    size_t mismatches = 0;
    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) {
            for(uint32_t c = 0; c < channels; ++c) mismatches += out.at(x, y, c) != expected(x, y, c);
        }
    }
    return mismatches;
}

}  // namespace

TEST_CASE("ImageManip resets the operations of the previous configuration", "[ImageManip]") {
    const auto input = makePacked(ImgFrame::Type::RGB888i, 64, 48);
    Manip manip({});

    ImageManipConfig convert;
    convert.setFrameType(ImgFrame::Type::BGR888i);
    ImageManipConfig crop;
    crop.addCrop(8, 4, 32, 24);
    ImageManipConfig passthrough;

    // Every configuration must give the same result, whatever ran before it
    for(int round = 0; round < 2; ++round) {
        auto cropped = run(manip, crop, input);
        REQUIRE(cropped);
        REQUIRE(cropped->type == ImgFrame::Type::RGB888i);
        REQUIRE(cropped->specs.width == 32);
        REQUIRE(cropped->specs.height == 24);
        CHECK(countMismatches(*cropped, 32, 24, 3, [](uint32_t x, uint32_t y, uint32_t c) { return pattern(x + 8, y + 4, c); }) == 0);

        auto converted = run(manip, convert, input);
        REQUIRE(converted);
        REQUIRE(converted->type == ImgFrame::Type::BGR888i);
        REQUIRE(converted->specs.width == 64);
        REQUIRE(converted->specs.height == 48);
        CHECK(countMismatches(*converted, 64, 48, 3, [](uint32_t x, uint32_t y, uint32_t c) { return pattern(x, y, 2 - c); }) == 0);

        auto same = run(manip, passthrough, input);
        REQUIRE(same);
        REQUIRE(same->type == ImgFrame::Type::RGB888i);
        REQUIRE(same->specs.width == 64);
        REQUIRE(same->specs.height == 48);
        CHECK(countMismatches(*same, 64, 48, 3, pattern) == 0);
    }
}

TEST_CASE("ImageManip rebuilds when only the input stride changes", "[ImageManip]") {
    const auto type = GENERATE(ImgFrame::Type::RGB888i, ImgFrame::Type::GRAY8);
    const uint32_t channels = type == ImgFrame::Type::RGB888i ? 3 : 1;
    const auto inverted = [](uint32_t x, uint32_t y, uint32_t c) { return static_cast<uint8_t>(255 - pattern(x, y, c)); };
    const auto tight = makePacked(type, 64, 48);
    // Same size and type, but rows padded to 100 pixels
    const auto padded = makePacked(type, 64, 48, inverted, 100 * channels);

    Manip manip({});
    ImageManipConfig crop;
    crop.addCrop(8, 4, 32, 24);
    for(const auto* input : {&tight, &padded, &tight}) {
        const PixelFn source = input == &padded ? PixelFn(inverted) : PixelFn(pattern);
        auto out = run(manip, crop, *input);
        REQUIRE(out);
        REQUIRE(out->specs.width == 32);
        REQUIRE(out->specs.height == 24);
        CHECK(countMismatches(*out, 32, 24, channels, [&](uint32_t x, uint32_t y, uint32_t c) { return source(x + 8, y + 4, c); }) == 0);
    }
}

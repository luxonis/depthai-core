// Regression tests for the ImageManip implementation shared by the host node and the RVC4 firmware.
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

/// Channels of a packed (interleaved / single channel) type
uint32_t packedChannels(ImgFrame::Type type) {
    return type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i ? 3 : 1;
}

struct Image {
    ImgFrame::Type type;
    FrameSpecs specs;
    std::shared_ptr<_ImageManipMemory> data;

    uint8_t* bytes() const {
        return data->getOffsetData().data();
    }
    /// Packed (interleaved / single channel) pixel
    uint8_t at(uint32_t x, uint32_t y, uint32_t c = 0) const {
        return bytes()[specs.p1Offset + y * specs.p1Stride + x * packedChannels(type) + c];
    }
};

/// A pattern without any symmetry, so that misplaced pixels always show
uint8_t pattern(uint32_t x, uint32_t y, uint32_t c) {
    return static_cast<uint8_t>((x * 7 + y * 13 + c * 61 + (x * y) % 17) & 0xFF);
}

/// Packed RGB888i / BGR888i / GRAY8 / RAW8 image without row padding, filled with pattern()
Image makePacked(ImgFrame::Type type, uint32_t width, uint32_t height) {
    const uint32_t channels = packedChannels(type);
    const uint32_t stride = width * channels;
    Image img{type, FrameSpecs{width, height, 0, 0, 0, stride, 0, 0}, std::make_shared<_ImageManipMemory>(stride * height)};
    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) {
            for(uint32_t c = 0; c < channels; ++c) img.bytes()[y * stride + x * channels + c] = pattern(x, y, c);
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
        // The node writes the source crops into ImgTransformations: the crop of the previous configuration must not leak
        CHECK(manip.getSrcCrops().empty());

        // run() hands back the input on passthrough, so check the manipulator's own state
        run(manip, passthrough, input);
        // The node forwards the input frame when the output size is 0
        CHECK(manip.getOutputSize() == 0);
        CHECK(manip.getOutputWidth() == 64);
        CHECK(manip.getOutputHeight() == 48);
        CHECK(manip.getOutputFrameType() == ImgFrame::Type::RGB888i);
    }
}

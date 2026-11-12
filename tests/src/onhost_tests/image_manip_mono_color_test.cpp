#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "depthai/utility/ImageManipImpl.hpp"

using dai::ImgFrame;
using namespace dai::impl;

TEST_CASE("ImageManip converts full-range mono to RGB and BGR without touching padding", "[ImageManip][ColorConversion]") {
    const auto inputType = GENERATE(ImgFrame::Type::RAW8, ImgFrame::Type::GRAY8);
    const auto outputType = GENERATE(ImgFrame::Type::RGB888i, ImgFrame::Type::BGR888i, ImgFrame::Type::RGB888p, ImgFrame::Type::BGR888p);
    const bool packedSource = GENERATE(false, true);
    const bool packedDestination = GENERATE(false, true);
    const bool planar = outputType == ImgFrame::Type::RGB888p || outputType == ImgFrame::Type::BGR888p;
    // Odd width, row padding, separate plane strides/offsets, and memory-view offsets.
    const FrameSpecs srcSpecs{5, 2, 3, 0, 0, packedSource ? 5U : 8U, 0, 0};
    const FrameSpecs dstSpecs{5, 2, 5, 31, 59, packedDestination ? (planar ? 5U : 15U) : (planar ? 9U : 19U), 5, packedDestination ? 5U : 11U};
    constexpr size_t memoryOffset = 7;
    constexpr uint8_t sentinel = 0xA5;
    const size_t sourceSize = srcSpecs.p1Offset + (srcSpecs.height - 1) * srcSpecs.p1Stride + srcSpecs.width;
    auto src = std::make_shared<_ImageManipMemory>(memoryOffset + sourceSize);
    auto dst = std::make_shared<_ImageManipMemory>(100);
    std::fill(src->getData().begin(), src->getData().end(), sentinel);
    std::fill(dst->getData().begin(), dst->getData().end(), sentinel);
    src->setOffset(memoryOffset);
    dst->setOffset(memoryOffset);
    const std::array<uint8_t, 10> samples{0, 1, 16, 127, 255, 235, 254, 128, 64, 32};
    std::vector<uint8_t> expected(100, sentinel);
    const std::array<uint32_t, 3> offsets{dstSpecs.p1Offset, dstSpecs.p2Offset, dstSpecs.p3Offset};
    const std::array<uint32_t, 3> strides{dstSpecs.p1Stride, dstSpecs.p2Stride, dstSpecs.p3Stride};
    for(uint32_t y = 0; y < srcSpecs.height; ++y) {
        for(uint32_t x = 0; x < srcSpecs.width; ++x) {
            const uint8_t value = samples[y * srcSpecs.width + x];
            src->getOffsetData()[srcSpecs.p1Offset + y * srcSpecs.p1Stride + x] = value;
            for(size_t c = 0; c < 3; ++c) {
                const size_t pos = planar ? offsets[c] + y * strides[c] + x : dstSpecs.p1Offset + y * dstSpecs.p1Stride + 3 * x + c;
                expected[memoryOffset + pos] = value;
            }
        }
    }
    // Exercise the dispatcher used by ImageManip, including the RAW8/GRAY8 source type.
    ColorChangeH conversion;
    conversion.build(srcSpecs, dstSpecs, inputType, outputType);
    conversion.apply(src, dst);
    const auto result = dst->getData();
    REQUIRE(std::vector<uint8_t>(result.begin(), result.end()) == expected);
}

TEST_CASE("ImageManip rejects invalid mono source views before writing color output", "[ImageManip][ColorConversion]") {
    const auto inputType = GENERATE(ImgFrame::Type::RAW8, ImgFrame::Type::GRAY8);
    const auto outputType = GENERATE(ImgFrame::Type::RGB888i, ImgFrame::Type::BGR888i, ImgFrame::Type::RGB888p, ImgFrame::Type::BGR888p);
    FrameSpecs srcSpecs{5, 2, 3, 0, 0, 5, 0, 0};
    const FrameSpecs dstSpecs{5, 2, 0, 10, 20, outputType == ImgFrame::Type::RGB888p || outputType == ImgFrame::Type::BGR888p ? 5U : 15U, 5, 5};
    size_t viewSize = 10;
    SECTION("truncated pixel data") {
        // stride * height fits, but the plane offset puts the last three pixels outside the view.
    }
    SECTION("truncated padded row") {
        srcSpecs.p1Stride = 8;
        viewSize = 15;
    }
    SECTION("plane offset at the end") {
        srcSpecs.p1Offset = static_cast<uint32_t>(viewSize);
    }
    SECTION("plane offset beyond the end") {
        srcSpecs.p1Offset = std::numeric_limits<uint32_t>::max();
    }
    SECTION("stride smaller than width") {
        srcSpecs.p1Stride = 4;
        viewSize = 64;
    }
    SECTION("zero width and stride") {
        srcSpecs.width = 0;
        srcSpecs.p1Stride = 0;
    }
    SECTION("zero height") {
        srcSpecs.height = 0;
    }
    SECTION("row extent exceeds 32-bit arithmetic") {
        srcSpecs.width = 1;
        srcSpecs.height = 65537;
        srcSpecs.p1Stride = 65536;
    }
    constexpr size_t memoryOffset = 7;
    // Keep backing storage larger than the logical view so the pre-fix truncated-view
    // regression can fail without accessing memory outside the allocation.
    auto src = std::make_shared<_ImageManipMemory>(100);
    src->setSize(memoryOffset + viewSize);
    src->setOffset(memoryOffset);
    auto dst = std::make_shared<_ImageManipMemory>(30);
    constexpr uint8_t sentinel = 0xA5;
    std::fill(dst->getData().begin(), dst->getData().end(), sentinel);
    ColorChangeH conversion;
    conversion.build(srcSpecs, dstSpecs, inputType, outputType);
    REQUIRE_THROWS_AS(conversion.apply(src, dst), std::runtime_error);
    REQUIRE(std::all_of(dst->getData().begin(), dst->getData().end(), [sentinel](uint8_t value) { return value == sentinel; }));
}

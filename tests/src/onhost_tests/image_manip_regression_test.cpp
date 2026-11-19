// Regression tests for the ImageManip implementation shared by the host node and the RVC4 firmware.
#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <vector>

#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/matrixOps.hpp"

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

/// NV12 image with the given luma and per chroma sample U / V values
Image makeNV12(uint32_t width,
               uint32_t height,
               const std::function<uint8_t(uint32_t, uint32_t)>& luma,
               const std::function<uint8_t(uint32_t, uint32_t)>& u,
               const std::function<uint8_t(uint32_t, uint32_t)>& v) {
    Image img{ImgFrame::Type::NV12,
              FrameSpecs{width, height, 0, width * height, width * height, width, width, width},
              std::make_shared<_ImageManipMemory>(width * height * 3 / 2)};
    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) img.bytes()[y * width + x] = luma(x, y);
    }
    for(uint32_t y = 0; y < height / 2; ++y) {
        for(uint32_t x = 0; x < width / 2; ++x) {
            img.bytes()[width * height + y * width + 2 * x] = u(x, y);
            img.bytes()[width * height + y * width + 2 * x + 1] = v(x, y);
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

struct MoveCheck {
    size_t covered = 0;     // output pixels whose centre maps onto a source pixel centre
    size_t mismatches = 0;  // ... and that differ from that source pixel
};

/// For transforms that move whole pixels (flips, 90 degree rotations, integer crops): every output pixel whose centre maps onto
/// the centre of a source pixel must equal that pixel exactly. ImageManip matrices use the corner convention (pixel i spans [i, i + 1]).
MoveCheck checkExactMove(const Manip& manip, const Image& out, const Image& in, uint32_t channels) {
    const auto inv = dai::matrix::getMatrixInverse(manip.getMatrix());
    MoveCheck result;
    size_t offGrid = 0;
    for(uint32_t y = 0; y < out.specs.height; ++y) {
        for(uint32_t x = 0; x < out.specs.width; ++x) {
            const float cx = x + 0.5f, cy = y + 0.5f;
            const float sx = inv[0][0] * cx + inv[0][1] * cy + inv[0][2] - 0.5f;
            const float sy = inv[1][0] * cx + inv[1][1] * cy + inv[1][2] - 0.5f;
            const long ix = std::lround(sx), iy = std::lround(sy);
            offGrid += std::abs(sx - ix) >= 1e-3f || std::abs(sy - iy) >= 1e-3f;
            if(ix < 0 || iy < 0 || ix >= (long)in.specs.width || iy >= (long)in.specs.height) continue;
            ++result.covered;
            for(uint32_t c = 0; c < channels; ++c) result.mismatches += out.at(x, y, c) != in.at(ix, iy, c);
        }
    }
    REQUIRE(offGrid == 0);  // The transform must move whole pixels
    return result;
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

TEST_CASE("ImageManip converts single channel images to YUV", "[ImageManip][ColorConversion]") {
    const auto target = GENERATE(ImgFrame::Type::NV12, ImgFrame::Type::YUV420p);
    const auto gray = makePacked(ImgFrame::Type::GRAY8, 64, 48);
    // The same image as RGB, R = G = B
    const auto rgb = makePacked(ImgFrame::Type::RGB888i, 64, 48, [](uint32_t x, uint32_t y, uint32_t) { return pattern(x, y, 0); });

    ImageManipConfig convert;
    convert.setFrameType(target);
    auto fromGray = run(convert, gray);
    auto fromRgb = run(convert, rgb);
    REQUIRE(fromGray);
    REQUIRE(fromRgb);
    REQUIRE(fromGray->type == target);
    REQUIRE(fromGray->data->getOffsetSize() == fromRgb->data->getOffsetSize());
    const auto a = fromGray->data->getOffsetData();
    const auto b = fromRgb->data->getOffsetData();
    CHECK(std::equal(a.begin(), a.end(), b.begin()));
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

TEST_CASE("ImageManip colormaps single channel images to RGB888i", "[ImageManip][Colormap]") {
    const auto gray = makePacked(ImgFrame::Type::GRAY8, 64, 48);
    // Reference: OpenCV's JET colormap (BGR) of the full image, as RGB
    cv::Mat grayMat(48, 64, CV_8UC1, gray.bytes(), gray.specs.p1Stride);
    cv::Mat reference;
    cv::applyColorMap(grayMat, reference, cv::COLORMAP_JET);
    cv::cvtColor(reference, reference, cv::COLOR_BGR2RGB);

    ImageManipConfig plain;
    plain.addCrop(8, 4, 32, 24);
    ImageManipConfig colormap;
    colormap.setColormap(dai::Colormap::JET);
    ImageManipConfig cropColormap;
    cropColormap.addCrop(8, 4, 32, 24);
    cropColormap.setColormap(dai::Colormap::JET);

    // The output type must come from the configuration being built, not from the one before it
    Manip manip({});
    for(const auto* cfg : {&plain, &colormap, &cropColormap, &plain, &colormap}) {
        auto out = run(manip, *cfg, gray);
        REQUIRE(out);
        if(cfg == &plain) {
            CHECK(out->type == ImgFrame::Type::GRAY8);
            continue;
        }
        REQUIRE(out->type == ImgFrame::Type::RGB888i);
        const uint32_t x0 = cfg == &cropColormap ? 8 : 0;
        const uint32_t y0 = cfg == &cropColormap ? 4 : 0;
        const uint32_t width = cfg == &cropColormap ? 32 : 64;
        const uint32_t height = cfg == &cropColormap ? 24 : 48;
        REQUIRE(out->specs.width == width);
        REQUIRE(out->specs.height == height);
        CHECK(countMismatches(*out, width, height, 3, [&](uint32_t x, uint32_t y, uint32_t c) { return reference.at<cv::Vec3b>(y + y0, x + x0)[c]; }) == 0);
    }
}

TEST_CASE("ImageManip warps NV12 chroma with the plane's own matrix", "[ImageManip][NV12]") {
    constexpr uint32_t width = 160, height = 120;
    // Chroma gradients: U encodes the chroma column, V the chroma row
    const auto input = makeNV12(
        width,
        height,
        [](uint32_t x, uint32_t y) { return static_cast<uint8_t>(x + y); },
        [](uint32_t x, uint32_t) { return static_cast<uint8_t>(2 * x + 20); },
        [](uint32_t, uint32_t y) { return static_cast<uint8_t>(2 * y + 20); });

    ImageManipConfig cfg;
    cfg.addTransformPerspective({1.0f, 0.05f, 4.0f, 0.03f, 1.0f, 2.0f, 0.0015f, 0.001f, 1.0f});
    Manip manip({});
    auto out = run(manip, cfg, input);
    REQUIRE(out);
    REQUIRE(out->type == ImgFrame::Type::NV12);
    const auto inv = dai::matrix::getMatrixInverse(manip.getMatrix());

    size_t checked = 0, wrong = 0;
    for(uint32_t j = 0; j < out->specs.height / 2; ++j) {
        for(uint32_t i = 0; i < out->specs.width / 2; ++i) {
            // Centre of the chroma sample's 2x2 luma block, mapped back to the source frame
            const float px = 2.0f * i + 1.0f, py = 2.0f * j + 1.0f;
            const float w = inv[2][0] * px + inv[2][1] * py + inv[2][2];
            const float sx = (inv[0][0] * px + inv[0][1] * py + inv[0][2]) / w;
            const float sy = (inv[1][0] * px + inv[1][1] * py + inv[1][2]) / w;
            if(sx < 4 || sy < 4 || sx > width - 4 || sy > height - 4) continue;  // Background or edge
            const float expectedU = 2.0f * (sx - 1.0f) / 2.0f + 20.0f;
            const float expectedV = 2.0f * (sy - 1.0f) / 2.0f + 20.0f;
            const uint8_t* uv = out->bytes() + out->specs.p2Offset + j * out->specs.p2Stride + 2 * i;
            ++checked;
            wrong += std::abs(uv[0] - expectedU) > 3.0f || std::abs(uv[1] - expectedV) > 3.0f;
        }
    }
    REQUIRE(checked > 1000);
    CHECK(wrong == 0);
}

TEST_CASE("ImageManip flips and rotates by whole pixels", "[ImageManip][Warp]") {
    const auto type = GENERATE(ImgFrame::Type::RGB888i, ImgFrame::Type::GRAY8);
    const uint32_t channels = type == ImgFrame::Type::RGB888i ? 3 : 1;
    const auto input = makePacked(type, 64, 48);

    ImageManipConfig flipH, flipV, rot90, rot180, rot270;
    flipH.addFlipHorizontal();
    flipV.addFlipVertical();
    rot90.addRotateDeg(90);
    rot180.addRotateDeg(180);
    rot270.addRotateDeg(270);
    for(const auto* cfg : {&flipH, &flipV, &rot90, &rot180, &rot270}) {
        Manip manip({});
        auto out = run(manip, *cfg, input);
        REQUIRE(out);
        const auto check = checkExactMove(manip, *out, input, channels);
        CHECK(check.covered == 64 * 48);
        CHECK(check.mismatches == 0);
    }
}

TEST_CASE("ImageManip crops before flipping and rotating", "[ImageManip][Warp]") {
    const auto input = makePacked(ImgFrame::Type::RGB888i, 64, 48);
    ImageManipConfig cropFlip, cropRotate, cropLetterbox;
    cropFlip.addCrop(10, 6, 20, 16).addFlipHorizontal();
    cropRotate.addCrop(10, 6, 20, 16).addRotateDeg(90);
    cropLetterbox.addCrop(10, 6, 20, 16).setOutputSize(30, 30, ImageManipConfig::ResizeMode::NONE);
    for(const auto* cfg : {&cropFlip, &cropRotate, &cropLetterbox}) {
        Manip manip({});
        auto out = run(manip, *cfg, input);
        REQUIRE(out);
        // Only the crop's content is moved: pixels mapping outside the crop window are background
        const auto inv = dai::matrix::getMatrixInverse(manip.getMatrix());
        size_t covered = 0, mismatches = 0;
        for(uint32_t y = 0; y < out->specs.height; ++y) {
            for(uint32_t x = 0; x < out->specs.width; ++x) {
                const float cx = x + 0.5f, cy = y + 0.5f;
                const long sx = std::lround(inv[0][0] * cx + inv[0][1] * cy + inv[0][2] - 0.5f);
                const long sy = std::lround(inv[1][0] * cx + inv[1][1] * cy + inv[1][2] - 0.5f);
                if(sx < 10 || sy < 6 || sx >= 30 || sy >= 22) continue;
                ++covered;
                for(uint32_t c = 0; c < 3; ++c) mismatches += out->at(x, y, c) != input.at(sx, sy, c);
            }
        }
        CHECK(covered == 20 * 16);
        CHECK(mismatches == 0);
    }
}

TEST_CASE("ImageManip upscales to the content edge without background bleeding in", "[ImageManip][Warp]") {
    // A linear ramp: bilinear interpolation reproduces it exactly, so any background blended into the edges shows
    const auto ramp = [](uint32_t x, uint32_t y, uint32_t c) { return static_cast<uint8_t>(40 + 3 * x + 2 * y + 20 * c); };
    const auto input = makePacked(ImgFrame::Type::RGB888i, 64, 48, ramp);
    ImageManipConfig cfg;
    cfg.addCrop(8, 8, 16, 12).addScale(2.0f);
    cfg.setBackgroundColor(255, 0, 255);
    Manip manip({});
    auto out = run(manip, cfg, input);
    REQUIRE(out);
    REQUIRE(out->specs.width == 32);
    REQUIRE(out->specs.height == 24);
    size_t wrong = 0;
    for(uint32_t y = 0; y < 24; ++y) {
        for(uint32_t x = 0; x < 32; ++x) {
            // Output pixel centre -> source pixel coordinates, clamped to the crop (edge pixels replicate the content edge)
            const float sx = std::clamp((x + 0.5f) / 2.0f - 0.5f, 0.0f, 15.0f) + 8.0f;
            const float sy = std::clamp((y + 0.5f) / 2.0f - 0.5f, 0.0f, 11.0f) + 8.0f;
            for(uint32_t c = 0; c < 3; ++c) wrong += std::abs(out->at(x, y, c) - (40.0f + 3.0f * sx + 2.0f * sy + 20.0f * c)) > 1.0f;
        }
    }
    CHECK(wrong == 0);
}

TEST_CASE("ImageManip YUV420p output can be read back", "[ImageManip][YUV420p]") {
    // Odd chroma height (35 rows) and a width that needs stride padding
    constexpr uint32_t width = 100, height = 70;
    const auto smooth = [](uint32_t x, uint32_t y, uint32_t c) { return static_cast<uint8_t>(20 + x + y + 30 * c); };  // <= 248
    const auto input = makePacked(ImgFrame::Type::RGB888i, width, height, smooth);
    ImageManipConfig cfg;
    cfg.setFrameType(ImgFrame::Type::YUV420p);
    auto out = run(cfg, input);
    REQUIRE(out);
    REQUIRE(out->type == ImgFrame::Type::YUV420p);

    dai::ImgFrame frame;
    frame.setData(std::vector<uint8_t>(out->bytes(), out->bytes() + out->data->getOffsetSize()));
    frame.setType(ImgFrame::Type::YUV420p).setWidth(width).setHeight(height).setStride(out->specs.p1Stride);
    frame.fb.p1Offset = out->specs.p1Offset;
    frame.fb.p2Offset = out->specs.p2Offset;
    frame.fb.p3Offset = out->specs.p3Offset;

    // ImgFrame carries a single stride: the chroma stride is derived from the plane offsets
    const auto readSpecs = getSrcFrameSpecs(frame.fb);
    CHECK(readSpecs.p2Stride == out->specs.p2Stride);
    CHECK(readSpecs.p3Stride == out->specs.p3Stride);
    CHECK(readSpecs.p3Offset - readSpecs.p2Offset == out->specs.p2Stride * (height / 2));

    const cv::Mat bgr = frame.getCvFrame();
    REQUIRE(bgr.rows == (int)height);
    REQUIRE(bgr.cols == (int)width);
    size_t wrong = 0;
    for(uint32_t y = 0; y < height; ++y) {
        for(uint32_t x = 0; x < width; ++x) {
            for(uint32_t c = 0; c < 3; ++c) wrong += std::abs(bgr.at<cv::Vec3b>(y, x)[2 - c] - smooth(x, y, c)) > 6;
        }
    }
    CHECK(wrong == 0);
}

TEST_CASE("ImageManip stores source crops in source coordinates", "[ImageManip][Transformation]") {
    // The input frame is a 2x downscale of a 200x200 source (e.g. an ISP scale)
    dai::ImgTransformation inputTransformation(200, 200);
    inputTransformation.addScale(0.5f, 0.5f);
    inputTransformation.setSize(100, 100);

    const auto input = makePacked(ImgFrame::Type::GRAY8, 100, 100);
    ImageManipConfig cfg;
    cfg.addCrop(10, 10, 20, 20);
    Manip manip({});
    REQUIRE(run(manip, cfg, input));

    // As the ImageManip node does it
    dai::ImgTransformation outputTransformation = inputTransformation;
    outputTransformation.addSrcCrops(manip.getSrcCrops(inputTransformation.getMatrixInv()));
    outputTransformation.addTransformation(manip.getMatrix());
    outputTransformation.setSize(20, 20);

    const auto crops = outputTransformation.getSrcCrops();
    REQUIRE(!crops.empty());
    CHECK(crops.back().center.x == Catch::Approx(40.0f));
    CHECK(crops.back().center.y == Catch::Approx(40.0f));
    CHECK(crops.back().size.width == Catch::Approx(40.0f));
    CHECK(crops.back().size.height == Catch::Approx(40.0f));
    // The whole output is cropped content
    for(size_t y : {1, 10, 18}) {
        for(size_t x : {1, 10, 18}) CHECK(outputTransformation.getDstMaskPt(x, y));
    }
}

TEST_CASE("ImageManip does not cut auto-sized outputs larger than 4000x3000", "[ImageManip][Size]") {
    const auto wide = makePacked(ImgFrame::Type::GRAY8, 5000, 64);
    const auto tall = makePacked(ImgFrame::Type::GRAY8, 64, 3500);
    ImageManipConfig flip, rotate;
    flip.addFlipHorizontal();
    rotate.addRotateDeg(90);
    for(const auto& [cfg, input] : {std::pair{&flip, &wide}, std::pair{&rotate, &wide}, std::pair{&flip, &tall}}) {
        Manip manip({});
        auto out = run(manip, *cfg, *input);
        REQUIRE(out);
        const bool swapped = cfg == &rotate;
        CHECK(out->specs.width == (swapped ? input->specs.height : input->specs.width));
        CHECK(out->specs.height == (swapped ? input->specs.width : input->specs.height));
        const auto check = checkExactMove(manip, *out, *input, 1);
        CHECK(check.covered == (size_t)input->specs.width * input->specs.height);
        CHECK(check.mismatches == 0);
    }
}

TEST_CASE("ImageManip keeps the full frame when rotating by 90 degrees", "[ImageManip][Size]") {
    // Sizes where the rotated corners come out of the float math just off integral
    const auto [width, height] = GENERATE(std::pair{1280u, 800u}, std::pair{640u, 480u}, std::pair{1920u, 1080u}, std::pair{416u, 416u});
    const auto input = makePacked(ImgFrame::Type::GRAY8, width, height);
    const auto angle = GENERATE(90.0f, 270.0f);
    ImageManipConfig rotate;
    rotate.addRotateDeg(angle);
    Manip manip({});
    auto out = run(manip, rotate, input);
    REQUIRE(out);
    CHECK(out->specs.width == height);
    CHECK(out->specs.height == width);
    const auto check = checkExactMove(manip, *out, input, 1);
    CHECK(check.covered == (size_t)width * height);
    CHECK(check.mismatches == 0);
}

TEST_CASE("ImageManip centres perspective and sheared content on its bounding box", "[ImageManip][Size]") {
    const auto [width, height] = GENERATE(std::pair{640u, 480u}, std::pair{1280u, 800u}, std::pair{1920u, 1080u});
    const float w = static_cast<float>(width), h = static_cast<float>(height);
    const auto input = makePacked(ImgFrame::Type::GRAY8, width, height);
    ImageManipConfig perspective, shear;
    perspective.addTransformPerspective({0.95f, 0.04f, 0.02f * w, -0.03f, 0.97f, 0.01f * h, 0.08f / w, 0.05f / h, 1.0f});
    perspective.setOutputSize(width, height, ImageManipConfig::ResizeMode::NONE);
    shear.addTransformAffine({1.0f, 0.3f, 0.0f, 1.0f});
    shear.setOutputSize(width, height, ImageManipConfig::ResizeMode::NONE);
    for(const auto* cfg : {&perspective, &shear}) {
        Manip manip({});
        REQUIRE(manip.build(cfg->base, cfg->outputFrameType, input.specs, input.type).getOutputWidth() == width);
        // The transformed image corners must be centred in the output
        const auto m = manip.getMatrix();
        float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
        for(const auto& corner : std::array<std::array<float, 2>, 4>{{{0, 0}, {w, 0}, {w, h}, {0, h}}}) {
            const auto p = matvecmul(m, corner);
            minX = std::min(minX, p[0]);
            maxX = std::max(maxX, p[0]);
            minY = std::min(minY, p[1]);
            maxY = std::max(maxY, p[1]);
        }
        CHECK(minX + maxX == Catch::Approx(w).margin(0.01));
        CHECK(minY + maxY == Catch::Approx(h).margin(0.01));
    }
}

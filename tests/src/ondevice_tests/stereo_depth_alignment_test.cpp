#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

namespace {
constexpr int WIDTH = 1280;
constexpr int HEIGHT = 720;
constexpr float FOCAL = 800.0f;
constexpr float BASELINE_CM = 7.5f;
constexpr int BACKGROUND_DISPARITY = 8;
using Matrix = std::array<std::array<float, 3>, 3>;

struct Square {
    int x;
    int y;
    int side;
    int disparity;
};

// Asymmetric placement and different depths detect shifts, zoom, cropping and swaps.
constexpr std::array<Square, 2> SQUARES = {{{256, 128, 128, 32}, {896, 448, 128, 16}}};

Matrix intrinsics(float scale = 1.0f, float shiftX = 0.0f, float shiftY = 0.0f) {
    return {{{FOCAL * scale, 0, WIDTH * scale / 2 + shiftX}, {0, FOCAL * scale, HEIGHT * scale / 2 + shiftY}, {0, 0, 1}}};
}

dai::CalibrationHandler calibration() {
    dai::CalibrationHandler result;
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const std::vector<std::vector<float>> k = {{FOCAL, 0, WIDTH / 2.0f}, {0, FOCAL, HEIGHT / 2.0f}, {0, 0, 1}};
    for(const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        result.setCameraIntrinsics(socket, k, WIDTH, HEIGHT);
        result.setDistortionCoefficients(socket, std::vector<float>(14, 0.0f));
        result.setCameraType(socket, dai::CameraModel::Perspective);
    }
    result.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, identity, {-BASELINE_CM, 0, 0}, {-BASELINE_CM, 0, 0});
    result.setStereoLeft(dai::CameraBoardSocket::CAM_B, identity);
    result.setStereoRight(dai::CameraBoardSocket::CAM_C, identity);
    return result;
}

std::array<std::vector<std::uint8_t>, 2> stereoImages() {
    std::array<std::vector<std::uint8_t>, 2> images;
    for(auto& image : images) image.resize(WIDTH * HEIGHT);
    // Fixed integer PRNG rather than an implementation-dependent random distribution.
    std::uint32_t seed = 0x12345678;
    for(auto& pixel : images[0]) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        pixel = static_cast<std::uint8_t>(seed);
    }
    // Render the background first, then foreground squares (near surfaces occlude it).
    // A flat square is unsuitable for stereo matching: retain texture inside each square.
    for(int y = 0; y < HEIGHT; ++y) {
        for(int x = BACKGROUND_DISPARITY; x < WIDTH; ++x) {
            bool foreground = false;
            for(const auto& square : SQUARES) {
                if(x >= square.x && x < square.x + square.side && y >= square.y && y < square.y + square.side) foreground = true;
            }
            if(!foreground) images[1][y * WIDTH + x - BACKGROUND_DISPARITY] = images[0][y * WIDTH + x];
        }
    }
    for(const auto& square : SQUARES) {
        for(int y = square.y; y < square.y + square.side; ++y) {
            for(int x = square.x; x < square.x + square.side; ++x) {
                images[1][y * WIDTH + x - square.disparity] = images[0][y * WIDTH + x];
            }
        }
    }
    return images;
}

std::shared_ptr<dai::ImgFrame> frame(int width, int height, dai::CameraBoardSocket socket, const Matrix& k, const std::vector<std::uint8_t>& pixels) {
    auto result = std::make_shared<dai::ImgFrame>();
    result->setType(dai::ImgFrame::Type::GRAY8);
    result->setWidth(width);
    result->setHeight(height);
    result->setStride(width);
    result->setInstanceNum(static_cast<unsigned>(socket));
    result->setTransformation(dai::ImgTransformation(width, height, k));
    result->setData(pixels);
    result->setSequenceNum(1);
    result->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(1)));
    return result;
}

void checkSquarePixels(const dai::ImgFrame& depth, bool right, float scale, int shiftX, int shiftY) {
    REQUIRE(depth.getType() == dai::ImgFrame::Type::RAW16);
    const auto& bytes = depth.getData();
    const auto stride = depth.getStride();
    REQUIRE(stride >= depth.getWidth() * sizeof(std::uint16_t));
    REQUIRE(bytes.size() >= stride * depth.getHeight());
    for(const auto& square : SQUARES) {
        CAPTURE(square.x, square.y, square.disparity);
        // Independent oracle: render geometry, not the firmware's homography calculation.
        const float x0 = (square.x - (right ? square.disparity : 0)) * scale + shiftX;
        const float y0 = square.y * scale + shiftY;
        const float side = square.side * scale;
        const float expectedDepth = FOCAL * BASELINE_CM * 10 / square.disparity;
        // Census/SGM and occlusions affect boundaries. Assert interiors and reject misplaced
        // foreground pixels outside a small boundary band; zeros count as interior failures.
        const float margin = std::ceil(8 * scale) + 1;
        std::size_t interior = 0, correctInterior = 0, foreground = 0, misplaced = 0;
        double sumX = 0, sumY = 0;
        for(unsigned y = 0; y < depth.getHeight(); ++y) {
            for(unsigned x = 0; x < depth.getWidth(); ++x) {
                std::uint16_t value;
                std::memcpy(&value, bytes.data() + y * stride + x * sizeof(value), sizeof(value));
                const bool isSquare = std::abs(static_cast<float>(value) - expectedDepth) < expectedDepth * 0.05f;
                if(x >= x0 + margin && x < x0 + side - margin && y >= y0 + margin && y < y0 + side - margin) {
                    ++interior;
                    if(isSquare) ++correctInterior;
                }
                if(isSquare) {
                    ++foreground;
                    sumX += x;
                    sumY += y;
                    if(x < x0 - margin || x >= x0 + side + margin || y < y0 - margin || y >= y0 + side + margin) ++misplaced;
                }
            }
        }
        CAPTURE(interior, correctInterior, foreground, misplaced, x0, y0, side);
        REQUIRE(interior > 0);
        REQUIRE(correctInterior >= interior * 0.90);
        REQUIRE(foreground > 0);
        CHECK(misplaced <= foreground * 0.03);
        CHECK(std::abs(sumX / foreground - (x0 + (side - 1) / 2)) <= margin);
        CHECK(std::abs(sumY / foreground - (y0 + (side - 1) / 2)) <= margin);
    }
}

void runAlignment(int decimation, bool right, bool explicitSize, bool fullSize, bool shifted, bool socketOnly = false) {
    CAPTURE(decimation, right, explicitSize, fullSize, shifted, socketOnly);
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC2) {
        SKIP("StereoDepth inputAlignTo geometry is an RVC2 firmware test");
    }
    // Pipeline-local calibration only: never flash or modify the device's EEPROM.
    pipeline.setCalibrationData(calibration());
    const int divisor = fullSize ? 1 : decimation;
    const float scale = 1.0f / divisor;
    const int width = WIDTH / divisor, height = HEIGHT / divisor;
    const int shiftX = shifted ? 32 : 0, shiftY = shifted ? 16 : 0;
    const auto socket = right ? dai::CameraBoardSocket::CAM_C : dai::CameraBoardSocket::CAM_B;

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setInputResolution(WIDTH, HEIGHT);
    stereo->useHomographyRectification(true);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(false);
    stereo->setExtendedDisparity(false);
    stereo->setDisparityToDepthUseSpecTranslation(false);
    stereo->initialConfig->setConfidenceThreshold(245);
    auto& post = stereo->initialConfig->postProcessing;
    post.median = dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;
    post.holeFilling.enable = false;
    post.adaptiveMedianFilter.enable = false;
    post.spatialFilter.enable = false;
    post.temporalFilter.enable = false;
    post.speckleFilter.enable = false;
    post.decimationFilter.decimationFactor = decimation;
    post.decimationFilter.decimationMode = dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING;
    if(explicitSize) stereo->setOutputSize(width, height);
    auto leftQueue = stereo->left.createInputQueue();
    auto rightQueue = stereo->right.createInputQueue();
    std::shared_ptr<dai::InputQueue> alignQueue;
    if(socketOnly) {
        stereo->setDepthAlign(socket);
    } else {
        alignQueue = stereo->inputAlignTo.createInputQueue();
    }
    auto output = stereo->depth.createOutputQueue();
    pipeline.start();
    const auto images = stereoImages();
    if(alignQueue) {
        alignQueue->send(frame(width, height, socket, intrinsics(scale, shiftX, shiftY), std::vector<std::uint8_t>(width * height)));
    }
    leftQueue->send(frame(WIDTH, HEIGHT, dai::CameraBoardSocket::CAM_B, intrinsics(), images[0]));
    rightQueue->send(frame(WIDTH, HEIGHT, dai::CameraBoardSocket::CAM_C, intrinsics(), images[1]));
    bool timedOut = false;
    auto depth = output->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    REQUIRE(depth->getWidth() == static_cast<unsigned>(width));
    REQUIRE(depth->getHeight() == static_cast<unsigned>(height));
    checkSquarePixels(*depth, right, scale, shiftX, shiftY);
    pipeline.stop();
}
}  // namespace

TEST_CASE("RVC2 StereoDepth aligns synthetic depth squares to target pixels", "[stereo-alignment]") {
    const auto decimation = GENERATE(1, 2, 4);
    const auto right = GENERATE(false, true);
    const auto explicitSize = GENERATE(false, true);
    SECTION("Target matches decimated depth size") {
        runAlignment(decimation, right, explicitSize, false, false);
    }
    SECTION("Target retains original resolution") {
        runAlignment(decimation, right, explicitSize, true, false);
    }
    SECTION("Target has a different principal point") {
        runAlignment(decimation, right, explicitSize, false, true);
    }
}

TEST_CASE("RVC2 StereoDepth socket alignment preserves synthetic depth squares", "[stereo-alignment]") {
    const auto decimation = GENERATE(1, 2, 4);
    const auto right = GENERATE(false, true);
    runAlignment(decimation, right, true, false, false, true);
}

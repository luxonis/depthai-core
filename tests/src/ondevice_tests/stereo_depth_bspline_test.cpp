/**
 * RVC4 B-spline rectification on device.
 *
 * The synthetic cameras are pinhole (f = 800 px, 1280x800, 7.5 cm baseline). A 20x12 B-spline is fitted to the
 * same pinhole projection, so rectifying with BSPLINE must give the same depth as LEGACY. Both runs render textured
 * squares at known disparities and check depth, position and size of each square. The calibration exists only
 * in the pipeline: nothing is written to the device. Run with the Catch2 filter [bspline] on an RVC4 device.
 */
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "depthai/common/BsplineCalibration.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"

namespace {
constexpr int WIDTH = 1280;
constexpr int HEIGHT = 800;
constexpr double FOCAL = 800.0;
constexpr float BASELINE_CM = 7.5f;
constexpr int NX = 20, NY = 12;
constexpr double FOV_X_DEG = 106.0;
constexpr int BACKGROUND_DISPARITY = 8;

struct Square {
    int x, y, side, disparity;
};
constexpr std::array<Square, 2> SQUARES = {{{256, 128, 128, 32}, {896, 448, 128, 16}}};

const std::vector<std::vector<float>> K = {{float(FOCAL), 0, WIDTH / 2.0f}, {0, float(FOCAL), HEIGHT / 2.0f}, {0, 0, 1}};
const std::vector<std::vector<float>> IDENTITY = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

dai::CalibrationHandler pinholeCalibration() {
    dai::CalibrationHandler result;
    for(const auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        result.setCameraIntrinsics(socket, K, WIDTH, HEIGHT);
        result.setDistortionCoefficients(socket, std::vector<float>(14, 0.0f));
        result.setCameraType(socket, dai::CameraModel::Perspective);
    }
    result.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, IDENTITY, {-BASELINE_CM, 0, 0}, {-BASELINE_CM, 0, 0});
    result.setStereoLeft(dai::CameraBoardSocket::CAM_B, IDENTITY);
    result.setStereoRight(dai::CameraBoardSocket::CAM_C, IDENTITY);
    return result;
}

/** Least-squares spline whose projection equals the pinhole camera: linear in the control points for a fixed core. */
dai::BsplineCameraCalibration pinholeSpline() {
    const double density = (NX - 3) / (4 * std::tan(FOV_X_DEG * M_PI / 720));
    cv::Mat normal = cv::Mat::eye(NX * NY, NX * NY, CV_64F) * 1e-6, rhsX = cv::Mat::zeros(NX * NY, 1, CV_64F), rhsY = rhsX.clone();
    for(int py = 0; py < HEIGHT; py += 10) {
        for(int px = 0; px < WIDTH; px += 10) {
            const double x = (px - WIDTH / 2.0) / FOCAL, y = (py - HEIGHT / 2.0) / FOCAL, norm = std::sqrt(x * x + y * y + 1);
            const double ux = 2 * x / (norm + 1), uy = 2 * y / (norm + 1);
            const double ix = ux * density + (NX - 1) / 2.0, iy = uy * density + (NY - 1) / 2.0;
            const int x0 = static_cast<int>(std::clamp(ix, 1.0, NX - 3.0)), y0 = static_cast<int>(std::clamp(iy, 1.0, NY - 3.0));
            auto basis = [](double t) {
                return std::array<double, 4>{
                    {(1 - 3 * t + 3 * t * t - t * t * t) / 6, (4 - 6 * t * t + 3 * t * t * t) / 6, (1 + 3 * t + 3 * t * t - 3 * t * t * t) / 6, t * t * t / 6}};
            };
            const auto bx = basis(ix - x0), by = basis(iy - y0);
            std::array<std::pair<int, double>, 16> row;
            for(int j = 0; j < 4; ++j)
                for(int i = 0; i < 4; ++i) row[j * 4 + i] = {(y0 - 1 + j) * NX + x0 - 1 + i, by[j] * bx[i]};
            for(const auto& [a, wa] : row) {
                for(const auto& [b, wb] : row) normal.at<double>(a, b) += wa * wb;
                rhsX.at<double>(a) += wa * (x - ux);
                rhsY.at<double>(a) += wa * (y - uy);
            }
        }
    }
    cv::Mat dx, dy;
    cv::solve(normal, rhsX, dx, cv::DECOMP_CHOLESKY);
    cv::solve(normal, rhsY, dy, cv::DECOMP_CHOLESKY);
    dai::BsplineCameraCalibration spline;
    spline.nx = NX;
    spline.ny = NY;
    spline.fovXDegrees = FOV_X_DEG;
    spline.intrinsics = {FOCAL, FOCAL, WIDTH / 2.0, HEIGHT / 2.0};
    for(int i = 0; i < NX * NY; ++i) {
        spline.intrinsics.push_back(dx.at<double>(i));
        spline.intrinsics.push_back(dy.at<double>(i));
    }
    return spline;
}

dai::BsplineStereoCalibration pinholeStereoSpline(const std::string& deviceId) {
    const std::vector<double> identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    dai::BsplineStereoCalibration model;
    model.deviceId = deviceId;
    model.calibrationId = "pinhole-equivalent-test";
    model.leftSocket = dai::CameraBoardSocket::CAM_B;
    model.rightSocket = dai::CameraBoardSocket::CAM_C;
    model.width = WIDTH;
    model.height = HEIGHT;
    model.left = model.right = pinholeSpline();
    model.rotationLeft = model.rotationRight = model.leftToRightRotation = identity;
    model.leftToRightTranslationCm = {-BASELINE_CM, 0, 0};
    model.baselineCm = BASELINE_CM;
    model.virtualIntrinsics = {FOCAL, FOCAL, WIDTH / 2.0, HEIGHT / 2.0};
    model.validate();
    return model;
}

/** Textured background at a fixed disparity, with textured squares in front of it. */
std::array<std::vector<std::uint8_t>, 2> stereoImages() {
    std::array<std::vector<std::uint8_t>, 2> images;
    for(auto& image : images) image.resize(WIDTH * HEIGHT);
    std::uint32_t seed = 0x12345678;
    for(auto& pixel : images[0]) {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        pixel = static_cast<std::uint8_t>(seed);
    }
    for(int y = 0; y < HEIGHT; ++y) {
        for(int x = BACKGROUND_DISPARITY; x < WIDTH; ++x) images[1][y * WIDTH + x - BACKGROUND_DISPARITY] = images[0][y * WIDTH + x];
    }
    for(const auto& square : SQUARES) {
        for(int y = square.y; y < square.y + square.side; ++y) {
            for(int x = square.x; x < square.x + square.side; ++x) images[1][y * WIDTH + x - square.disparity] = images[0][y * WIDTH + x];
        }
    }
    return images;
}

std::shared_ptr<dai::ImgFrame> frame(dai::CameraBoardSocket socket, const std::vector<std::uint8_t>& pixels) {
    auto result = std::make_shared<dai::ImgFrame>();
    result->setType(dai::ImgFrame::Type::GRAY8);
    result->setWidth(WIDTH);
    result->setHeight(HEIGHT);
    result->setStride(WIDTH);
    result->setInstanceNum(static_cast<unsigned>(socket));
    std::array<std::array<float, 3>, 3> k{};
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) k[i][j] = K[i][j];
    result->setTransformation(dai::ImgTransformation(WIDTH, HEIGHT, k));
    result->setData(pixels);
    result->setSequenceNum(1);
    result->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::seconds(1)));
    return result;
}

/** Depth of each square's interior within 5 %, aligned to the right camera. */
void checkSquares(const dai::ImgFrame& depth) {
    REQUIRE(depth.getType() == dai::ImgFrame::Type::RAW16);
    REQUIRE(depth.getWidth() == static_cast<unsigned>(WIDTH));
    REQUIRE(depth.getHeight() == static_cast<unsigned>(HEIGHT));
    const auto& bytes = depth.getData();
    for(const auto& square : SQUARES) {
        CAPTURE(square.x, square.y, square.disparity);
        const float expected = FOCAL * BASELINE_CM * 10 / square.disparity;
        const int x0 = square.x - square.disparity, margin = 9;
        std::size_t interior = 0, correct = 0;
        for(int y = square.y + margin; y < square.y + square.side - margin; ++y) {
            for(int x = x0 + margin; x < x0 + square.side - margin; ++x) {
                std::uint16_t value;
                std::memcpy(&value, bytes.data() + y * depth.getStride() + x * sizeof(value), sizeof(value));
                ++interior;
                if(std::abs(value - expected) < expected * 0.05f) ++correct;
            }
        }
        CAPTURE(interior, correct, expected);
        REQUIRE(correct >= interior * 0.90);
    }
}

void runStereo(dai::StereoDepthProperties::RectificationModel model) {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("B-spline rectification is RVC4 only");
    auto calibration = pinholeCalibration();
    calibration.setBsplineStereoCalibration(pinholeStereoSpline(pipeline.getDefaultDevice()->getDeviceId()));
    pipeline.setCalibrationData(calibration);  // pipeline-local only, never the device's storage

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setInputResolution(WIDTH, HEIGHT);
    stereo->setRectificationModel(model);
    stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(false);
    stereo->initialConfig->setConfidenceThreshold(245);
    auto leftQueue = stereo->left.createInputQueue();
    auto rightQueue = stereo->right.createInputQueue();
    auto output = stereo->depth.createOutputQueue();
    pipeline.start();
    const auto images = stereoImages();
    leftQueue->send(frame(dai::CameraBoardSocket::CAM_B, images[0]));
    rightQueue->send(frame(dai::CameraBoardSocket::CAM_C, images[1]));
    bool timedOut = false;
    auto depth = output->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depth != nullptr);
    checkSquares(*depth);
    pipeline.stop();
}
}  // namespace

TEST_CASE("The pinhole-equivalent spline reproduces the pinhole projection", "[bspline]") {
    const auto spline = pinholeSpline();
    spline.validate();
    double worst = 0;
    for(int py = 0; py < HEIGHT; py += 7) {
        for(int px = 0; px < WIDTH; px += 7) {
            const auto p = spline.project({(px - WIDTH / 2.0) / FOCAL, (py - HEIGHT / 2.0) / FOCAL, 1.0});
            worst = std::max(worst, std::hypot(p[0] - px, p[1] - py));
        }
    }
    CAPTURE(worst);
    REQUIRE(worst < 0.15);  // ~0.09 px at the image corners; a wrong basis or knot index is off by whole pixels
}

TEST_CASE("StereoDepth gives the same synthetic depth with LEGACY and BSPLINE rectification", "[bspline]") {
    using Model = dai::StereoDepthProperties::RectificationModel;
    const auto model = GENERATE(Model::LEGACY, Model::BSPLINE);
    CAPTURE(static_cast<int>(model));
    runStereo(model);
}

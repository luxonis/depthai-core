#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <vector>

#include "depthai/depthai.hpp"
#include "pipeline/utilities/Alignment/RectifiedBounds.hpp"

namespace {
void runRotatedCoverageTest(bool runOnHost) {
    const std::array<std::array<float, 3>, 3> depthK = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    auto imageK = depthK;
    imageK[0][2] = 64.0f;
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation imageTransformation(128, 48, imageK, dai::CameraModel::Perspective, {}, imageExtrinsics);
    for(const float sign : {-1.0f, 1.0f}) {
        CAPTURE(sign);
        const float angle = sign * 20.0f * 3.14159265f / 180.0f;
        const std::vector<std::vector<float>> rotation = {
            {std::cos(angle), 0.0f, std::sin(angle)}, {0.0f, 1.0f, 0.0f}, {-std::sin(angle), 0.0f, std::cos(angle)}};
        const dai::Extrinsics depthExtrinsics(rotation, {sign * 70.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, depthK, dai::CameraModel::Perspective, {}, depthExtrinsics);

        dai::Pipeline pipeline(!runOnHost);
        auto align = pipeline.create<dai::node::Align>();
        align->setRunOnHost(runOnHost);
        auto depthQueue = align->input.createInputQueue();
        auto imageQueue = align->inputAlignTo.createInputQueue();
        auto outputQueue = align->outputAligned.createOutputQueue();

        auto depth = std::make_shared<dai::ImgFrame>();
        depth->setType(dai::ImgFrame::Type::RAW16);
        depth->setSize(64, 48);
        depth->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_B));
        depth->setTransformation(depthTransformation);
        std::vector<uint8_t> pixels(64 * 48 * 2);
        for(int y = 0; y < 48; ++y) {
            for(int x = 0; x < 64; ++x) {
                const uint16_t millimeters = 1000 + 7 * x + 11 * y;
                const size_t offset = 2 * (64 * y + x);
                pixels[offset] = static_cast<uint8_t>(millimeters);
                pixels[offset + 1] = static_cast<uint8_t>(millimeters >> 8);
            }
        }
        depth->setData(std::move(pixels));
        auto image = std::make_shared<dai::ImgFrame>();
        image->setType(dai::ImgFrame::Type::GRAY8);
        image->setSize(128, 48);
        image->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_A));
        image->setTransformation(imageTransformation);
        image->setData(std::vector<uint8_t>(128 * 48));

        pipeline.start();
        imageQueue->send(image);
        depthQueue->send(depth);
        bool timedOut = false;
        auto aligned = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        pipeline.stop();
        REQUIRE_FALSE(timedOut);
        REQUIRE(aligned != nullptr);
        REQUIRE(aligned->getWidth() == 128);
        REQUIRE(aligned->getHeight() == 48);
        REQUIRE(aligned->getType() == dai::ImgFrame::Type::RAW16);

        // Project an outer source point independently of the rectification maps.
        const auto r = depthTransformation.getRotationMatrixTo(imageTransformation);
        const auto t = depthTransformation.getTranslationVectorTo(imageTransformation, false, dai::LengthUnit::MILLIMETER);
        const float sourceX = sign > 0 ? 56.0f : 8.0f;
        const float z = 1000.0f + 7.0f * sourceX + 11.0f * 24.0f;
        const float x = (sourceX - 32.0f) * z / 64.0f;
        const float targetX = r[0][0] * x + r[0][2] * z + t[0];
        const float targetZ = r[2][0] * x + r[2][2] * z + t[2];
        const int u = static_cast<int>(std::lround(64.0f * targetX / targetZ + 64.0f));
        REQUIRE(u >= 2);
        REQUIRE(u < 126);
        const auto data = aligned->getData();
        REQUIRE(data.size() >= 128 * 48 * 2);
        for(int dx = -1; dx <= 1; ++dx) {
            const size_t offset = 2 * (24 * 128 + u + dx);
            const uint16_t millimeters = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
            CAPTURE(u, dx, millimeters);
            CHECK(millimeters > 1000);
        }
    }
}
}  // namespace

TEST_CASE("Test Align preserves rotated depth coverage on host") {
    runRotatedCoverageTest(true);
}

TEST_CASE("Test Align preserves rotated depth coverage on device") {
    runRotatedCoverageTest(false);
}

TEST_CASE("Test alignment rejects unsafe rectified bounds") {
    for(const auto& geometry : {std::pair<float, float>{20.0f, 40.0f}, {64.0f, 90.0f}}) {
        const auto [focalLength, yawDegrees] = geometry;
        CAPTURE(focalLength, yawDegrees);
        const cv::Mat k = (cv::Mat_<float>(3, 3) << focalLength, 0, 32, 0, focalLength, 24, 0, 0, 1);
        const cv::Mat distortion = cv::Mat::zeros(1, 14, CV_32F);
        const double angle = yawDegrees * CV_PI / 180.0;
        const cv::Mat rotation = (cv::Mat_<double>(3, 3) << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle));
        const cv::Mat translation = (cv::Mat_<double>(3, 1) << 70, 0, 0);
        cv::Mat r1, r2, p1, p2, q;
        cv::stereoRectify(k, distortion, k, distortion, {64, 48}, rotation, translation, r1, r2, p1, p2, q);
        REQUIRE_THROWS_AS(dai::detail::rectifiedBounds({64, 48}, k, distortion, r1, {64, 48}, k, r2), std::runtime_error);
    }
    // Chroma padding can exceed the pixel budget even when the unpadded size fits.
    REQUIRE_NOTHROW(dai::detail::validateRectifiedSize(4095, 4097));
    REQUIRE_THROWS_AS(dai::detail::validateRectifiedSize(4096, 4098), std::runtime_error);
    REQUIRE_THROWS_AS(dai::detail::validateRectifiedSize(0, 48), std::runtime_error);
    REQUIRE_THROWS_AS(dai::detail::validateRectifiedSize(16385, 1), std::runtime_error);
}

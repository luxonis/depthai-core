#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "depthai/depthai.hpp"
#include "pipeline/utilities/Alignment/RectifiedBounds.hpp"

namespace {
void runRotatedCoverageTest(bool runOnHost, bool distorted = false) {
    const std::array<std::array<float, 3>, 3> depthK = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    auto imageK = depthK;
    imageK[0][2] = 64.0f;
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics imageExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation imageTransformation(128, 48, imageK, dai::CameraModel::Perspective, {}, imageExtrinsics);
    const std::vector<float> distortion = distorted ? std::vector<float>{-8.0f / 27.0f, 0, 0, 0, 0} : std::vector<float>{};
    for(const float sign : {-1.0f, 1.0f}) {
        CAPTURE(sign);
        const float angle = sign * 20.0f * 3.14159265f / 180.0f;
        const std::vector<std::vector<float>> rotation = {
            {std::cos(angle), 0.0f, std::sin(angle)}, {0.0f, 1.0f, 0.0f}, {-std::sin(angle), 0.0f, std::cos(angle)}};
        const dai::Extrinsics depthExtrinsics(rotation, {sign * 70.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation depthTransformation(64, 48, depthK, dai::CameraModel::Perspective, distortion, depthExtrinsics);

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
        // This known ray lands exactly on an integer source pixel, with or without radial distortion.
        const float rayX = sign * 0.375f;
        const float sourceX = 32.0f + 64.0f * rayX * (1.0f + (distorted ? distortion[0] * rayX * rayX : 0.0f));
        REQUIRE(std::abs(sourceX - std::round(sourceX)) < 1e-4f);
        const float z = 1000.0f + 7.0f * sourceX + 11.0f * 24.0f;
        const float x = rayX * z;
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
void runDepthShiftTest(bool runOnHost) {
    const std::array<std::array<float, 3>, 3> k = {{{64.0f, 0.0f, 32.0f}, {0.0f, 64.0f, 24.0f}, {0.0f, 0.0f, 1.0f}}};
    const std::vector<std::vector<float>> identity = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const dai::Extrinsics targetExtrinsics(identity, {0.0f, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation targetTransformation(64, 48, k, dai::CameraModel::Perspective, {}, targetExtrinsics);
    constexpr uint16_t DEPTH = 2000;
    for(const auto& [baseline, displacement] : {std::pair<float, int>{0.0f, 0},
                                                {1.0f, 0},
                                                {-1.0f, 0},
                                                {15.3125f, 0},
                                                {-15.3125f, 0},
                                                {15.625f, 1},
                                                {-15.625f, -1},
                                                {15.9375f, 1},
                                                {-15.9375f, -1},
                                                {31.25f, 1},
                                                {-31.25f, -1},
                                                {140.0f, 4},
                                                {-140.0f, -4},
                                                {141.25f, 5},
                                                {-141.25f, -5}}) {
        CAPTURE(baseline, displacement);
        const dai::Extrinsics sourceExtrinsics(identity, {baseline, 0.0f, 0.0f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation sourceTransformation(64, 48, k, dai::CameraModel::Perspective, {}, sourceExtrinsics);
        dai::Pipeline pipeline(!runOnHost);
        auto align = pipeline.create<dai::node::Align>();
        align->setRunOnHost(runOnHost);
        auto inputQueue = align->input.createInputQueue();
        auto targetQueue = align->inputAlignTo.createInputQueue();
        auto outputQueue = align->outputAligned.createOutputQueue();
        auto input = std::make_shared<dai::ImgFrame>();
        input->setType(dai::ImgFrame::Type::RAW16);
        input->setSize(64, 48);
        input->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_B));
        input->setTransformation(sourceTransformation);
        std::vector<uint8_t> pixels(64 * 48 * 2, 0);
        pixels[2 * (24 * 64 + 32)] = static_cast<uint8_t>(DEPTH);
        pixels[2 * (24 * 64 + 32) + 1] = static_cast<uint8_t>(DEPTH >> 8);
        input->setData(std::move(pixels));
        auto target = std::make_shared<dai::ImgFrame>();
        target->setType(dai::ImgFrame::Type::GRAY8);
        target->setSize(64, 48);
        target->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_A));
        target->setTransformation(targetTransformation);
        target->setData(std::vector<uint8_t>(64 * 48));
        pipeline.start();
        targetQueue->send(target);
        inputQueue->send(input);
        bool timedOut = false;
        auto output = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        pipeline.stop();
        REQUIRE_FALSE(timedOut);
        REQUIRE(output != nullptr);
        REQUIRE(output->getWidth() == 64);
        REQUIRE(output->getHeight() == 48);
        const auto data = output->getData();
        REQUIRE(data.size() == 64 * 48 * 2);
        const int shiftedX = 32 + displacement;
        const int adjacentX = shiftedX + (baseline > 0 ? 1 : -1);
        for(int y = 0; y < 48; ++y) {
            for(int x = 0; x < 64; ++x) {
                CAPTURE(x, y);
                const size_t offset = 2 * (y * 64 + x);
                const uint16_t actual = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
                const uint16_t expected = y == 24 && (x == shiftedX || (displacement != 0 && x == adjacentX)) ? DEPTH : 0;
                CHECK(actual == expected);
            }
        }
    }
}
template <typename Node>
void runFourPixelProjectionTest(bool runOnHost) {
    constexpr int width = 64, height = 64;
    const std::array<std::array<float, 3>, 3> k = {{{32, 0, 32}, {0, 32, 32}, {0, 0, 1}}};
    const cv::Matx33d cameraMatrix(32, 0, 32, 0, 32, 32, 0, 0, 1);
    const std::vector<float> distortion = {0.5f, 0, 0, 0, 0};
    const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const dai::Extrinsics targetExtrinsics(identity, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation targetTransformation(width, height, k, dai::CameraModel::Perspective, {}, targetExtrinsics);
    for(const float baseline : {-70.0f, 70.0f}) {
        CAPTURE(baseline);
        const dai::Extrinsics sourceExtrinsics(identity, {baseline, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
        const dai::ImgTransformation sourceTransformation(width, height, k, dai::CameraModel::Perspective, distortion, sourceExtrinsics);
        auto input = std::make_shared<dai::ImgFrame>();
        input->setType(dai::ImgFrame::Type::RAW16);
        input->setSize(width, height);
        input->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_B));
        input->setTransformation(sourceTransformation);
        std::vector<uint8_t> pixels(width * height * 2, 0);
        const std::array<uint16_t, 4> depths = {1000, 2000, 4000, 8000};
        size_t marker = 0;
        for(const int y : {12, 52}) {
            for(const int x : {12, 52}) {
                const size_t offset = 2 * (y * width + x);
                pixels[offset] = static_cast<uint8_t>(depths[marker]);
                pixels[offset + 1] = static_cast<uint8_t>(depths[marker++] >> 8);
            }
        }
        input->setData(std::move(pixels));

        // Independently deproject the actual depth image; do not use the node's rectification maps.
        std::vector<cv::Point2d> observed, rays;
        std::vector<uint16_t> observedDepths;
        const auto& sourceData = input->getData();
        for(int y = 0; y < height; ++y) {
            for(int x = 0; x < width; ++x) {
                const size_t offset = 2 * (y * width + x);
                const uint16_t depth = sourceData[offset] | (static_cast<uint16_t>(sourceData[offset + 1]) << 8);
                if(depth == 0) continue;
                observed.emplace_back(x, y);
                observedDepths.push_back(depth);
            }
        }
        REQUIRE(observed.size() == 4);
        cv::undistortPoints(observed,
                            rays,
                            cameraMatrix,
                            distortion,
                            cv::noArray(),
                            cv::noArray(),
                            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-12));
        std::vector<cv::Point3d> points;
        for(size_t i = 0; i < rays.size(); ++i) {
            points.emplace_back(rays[i].x * observedDepths[i], rays[i].y * observedDepths[i], observedDepths[i]);
        }
        std::vector<cv::Point2d> projected;
        cv::projectPoints(points, cv::Vec3d(0, 0, 0), cv::Vec3d(baseline, 0, 0), cameraMatrix, cv::noArray(), projected);

        std::vector<uint16_t> expected(width * height, 0);
        for(size_t i = 0; i < projected.size(); ++i) {
            const int x = static_cast<int>(std::lround(projected[i].x));
            const int y = static_cast<int>(std::lround(projected[i].y));
            REQUIRE(x > 0);
            REQUIRE(x < width - 1);
            REQUIRE(y >= 0);
            REQUIRE(y < height);
            expected[y * width + x] = observedDepths[i];
            // Nonzero integer depth shifts retain the node's existing adjacent-pixel splat.
            const int sourceX = static_cast<int>(std::lround(32 * rays[i].x + 32));
            if(x != sourceX) expected[y * width + x + (baseline > 0 ? 1 : -1)] = observedDepths[i];
        }

        dai::Pipeline pipeline(!runOnHost);
        auto align = pipeline.create<Node>();
        align->setRunOnHost(runOnHost);
        auto inputQueue = align->input.createInputQueue();
        auto targetQueue = align->inputAlignTo.createInputQueue();
        std::shared_ptr<dai::MessageQueue> outputQueue = align->outputAligned.createOutputQueue();
        auto target = std::make_shared<dai::ImgFrame>();
        target->setType(dai::ImgFrame::Type::GRAY8);
        target->setSize(width, height);
        target->setInstanceNum(static_cast<unsigned int>(dai::CameraBoardSocket::CAM_A));
        target->setTransformation(targetTransformation);
        target->setData(std::vector<uint8_t>(width * height));
        pipeline.start();
        targetQueue->send(target);
        inputQueue->send(input);
        bool timedOut = false;
        auto output = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        pipeline.stop();
        REQUIRE_FALSE(timedOut);
        REQUIRE(output != nullptr);
        REQUIRE(output->getType() == dai::ImgFrame::Type::RAW16);
        REQUIRE(output->getWidth() == width);
        REQUIRE(output->getHeight() == height);
        const auto data = output->getData();
        REQUIRE(data.size() == width * height * 2);
        std::array<double, 4> nearest;
        nearest.fill(std::numeric_limits<double>::infinity());
        for(int y = 0; y < height; ++y) {
            for(int x = 0; x < width; ++x) {
                CAPTURE(x, y);
                const size_t index = y * width + x;
                const uint16_t actual = data[2 * index] | (static_cast<uint16_t>(data[2 * index + 1]) << 8);
                CHECK(actual == expected[index]);
                for(size_t i = 0; i < projected.size(); ++i) {
                    if(actual == observedDepths[i]) nearest[i] = std::min(nearest[i], cv::norm(cv::Point2d(x, y) - projected[i]));
                }
            }
        }
        for(size_t i = 0; i < nearest.size(); ++i) {
            CAPTURE(i, observedDepths[i], projected[i].x, projected[i].y, nearest[i]);
            CHECK(nearest[i] <= std::sqrt(0.5) + 1e-6);
        }
    }
}
}  // namespace

TEST_CASE("Test Align preserves rotated depth coverage on host") {
    runRotatedCoverageTest(true);
}

TEST_CASE("Test Align matches four distorted 3D projections on host") {
    runFourPixelProjectionTest<dai::node::Align>(true);
}

TEST_CASE("Test Align matches four distorted 3D projections on device") {
    runFourPixelProjectionTest<dai::node::Align>(false);
}

TEST_CASE("Test ImageAlign matches four distorted 3D projections on host") {
    runFourPixelProjectionTest<dai::node::ImageAlign>(true);
}

TEST_CASE("Test ImageAlign matches four distorted 3D projections on device") {
    runFourPixelProjectionTest<dai::node::ImageAlign>(false);
}

TEST_CASE("Test Align preserves rotated depth coverage on device") {
    runRotatedCoverageTest(false);
}

TEST_CASE("Test Align preserves distorted depth coverage on host") {
    runRotatedCoverageTest(true, true);
}

TEST_CASE("Test Align preserves distorted depth coverage on device") {
    runRotatedCoverageTest(false, true);
}

TEST_CASE("Test Align rounds signed shifts and only splats nonzero displacements on host") {
    runDepthShiftTest(true);
}

TEST_CASE("Test Align rounds signed shifts and only splats nonzero displacements on device") {
    runDepthShiftTest(false);
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

TEST_CASE("Test alignment rejects rectified rays behind the camera") {
    for(const int type : {CV_32F, CV_64F}) {
        CAPTURE(type);
        cv::Mat k = (cv::Mat_<double>(3, 3) << 64, 0, 32, 0, 64, 24, 0, 0, 1);
        k.convertTo(k, type);
        const cv::Mat identity = cv::Mat::eye(3, 3, type);
        for(const double yaw : {120.0, 180.0}) {
            CAPTURE(yaw);
            const double angle = yaw * CV_PI / 180;
            cv::Mat rotation = (cv::Mat_<double>(3, 3) << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle));
            rotation.convertTo(rotation, type);
            REQUIRE_THROWS_AS(dai::detail::rectifiedBounds({64, 48}, k, {}, rotation, {64, 48}, k, identity), std::runtime_error);
            REQUIRE_THROWS_AS(dai::detail::rectifiedBounds({64, 48}, k, {}, identity, {64, 48}, k, rotation), std::runtime_error);
        }
    }
}

TEST_CASE("Test alignment preserves bounds for forward rectified rays") {
    const cv::Mat k = (cv::Mat_<double>(3, 3) << 64, 0, 32, 0, 64, 24, 0, 0, 1);
    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat distortion = (cv::Mat_<double>(1, 5) << -0.1, 0.01, 0.001, -0.002, 0);
    std::vector<cv::Point2f> border;
    for(int x = 0; x <= 64; ++x) {
        border.emplace_back(x, 0);
        border.emplace_back(x, 48);
    }
    for(int y = 0; y <= 48; ++y) {
        border.emplace_back(0, y);
        border.emplace_back(64, y);
    }
    for(const double yaw : {-20.0, 0.0, 20.0}) {
        CAPTURE(yaw);
        const double angle = yaw * CV_PI / 180;
        const cv::Mat rotation = (cv::Mat_<double>(3, 3) << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle));
        std::vector<cv::Point2f> projected;
        cv::undistortPoints(border, projected, k, distortion, rotation, k, cv::TermCriteria(cv::TermCriteria::COUNT, 30, 0.01));
        const cv::Rect expected = cv::boundingRect(projected) | cv::Rect(0, 0, 65, 49);
        REQUIRE(dai::detail::rectifiedBounds({64, 48}, k, distortion, rotation, {64, 48}, k, identity) == expected);
    }
}

TEST_CASE("Test alignment rejects rectified rays too close to the horizon") {
    const cv::Mat sourceK = (cv::Mat_<double>(3, 3) << 1e-6, 0, 32, 0, 1e-6, 24, 0, 0, 1);
    const cv::Mat targetK = (cv::Mat_<double>(3, 3) << 1e9, 0, 32, 0, 1e9, 24, 0, 0, 1);
    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    const double angle = std::acos(5e-7);
    const cv::Mat rotation = (cv::Mat_<double>(3, 3) << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle));
    // The tiny projection focal length keeps pixel coordinates in range despite near-zero ray depth.
    REQUIRE_THROWS_AS(dai::detail::rectifiedBounds({64, 48}, sourceK, {}, identity, {64, 48}, targetK, rotation), std::runtime_error);
}

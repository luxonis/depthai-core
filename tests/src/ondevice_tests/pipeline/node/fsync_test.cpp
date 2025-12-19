#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

struct Thresholds {
    std::chrono::steady_clock::duration leftRight;
    std::chrono::steady_clock::duration all;
};

constexpr Thresholds RVC2_THRESHOLDS{std::chrono::microseconds(50), std::chrono::milliseconds(5)};
constexpr Thresholds RVC4_THRESHOLDS{std::chrono::microseconds(5), std::chrono::milliseconds(1)};

void testFsync(float fps, Thresholds thresholds, std::shared_ptr<dai::Device> device) {
    // Create pipeline
    dai::Pipeline p(std::move(device));
    auto rgb = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto sync = p.create<dai::node::Sync>();
    // Convert frame sync threshold to nanoseconds
    auto thresholdNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / fps));

    sync->setSyncThreshold(thresholdNs);
    left->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["left"]);
    right->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["right"]);
    rgb->requestOutput(std::make_pair(1280, 800), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["rgb"]);

    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(25);
    sync->out.link(benchmarkIn->input);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue(10, false);

    auto syncQueue = sync->out.createOutputQueue(10, false);
    p.start();

    for(int i = 0; i < 100; i++) {
        auto syncData = syncQueue->get<dai::MessageGroup>();
        REQUIRE(syncData != nullptr);

        auto leftFrame = syncData->get<dai::ImgFrame>("left");
        REQUIRE(leftFrame != nullptr);
        auto rightFrame = syncData->get<dai::ImgFrame>("right");
        REQUIRE(rightFrame != nullptr);
        auto rgbFrame = syncData->get<dai::ImgFrame>("rgb");
        REQUIRE(rgbFrame != nullptr);

        auto rgbTimestamp = rgbFrame->getTimestampDevice();
        auto leftTimestamp = leftFrame->getTimestampDevice();
        auto rightTimestamp = rightFrame->getTimestampDevice();

        // Compute the absolute difference between left and right timestamps.
        auto diffLeftRight = (leftTimestamp > rightTimestamp) ? (leftTimestamp - rightTimestamp) : (rightTimestamp - leftTimestamp);

        // Compute the difference between the maximum and minimum timestamps among all frames.
        auto maxTimestamp = std::max({leftTimestamp, rightTimestamp, rgbTimestamp});
        auto minTimestamp = std::min({leftTimestamp, rightTimestamp, rgbTimestamp});
        auto diffAll = maxTimestamp - minTimestamp;

        // Verify that the differences are within the expected thresholds.
        // 'thresholds.leftRight' applies to the left/right pair, while
        // 'thresholds.all' applies to the overall synchronization among all frames.
        REQUIRE(diffLeftRight <= thresholds.leftRight);
        REQUIRE(diffAll <= thresholds.all);
    }

    for(int i = 0; i < 4; i++) {
        auto reportData = benchmarkQueue->get<dai::BenchmarkReport>();
        if(i < 2) {
            // Skip the first two reports, to let the FPS stabilize.
            continue;
        }
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
        REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(0.1));
    }
}

void testFsyncIntegrated(float fps) {
    // Create device
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        testFsync(fps, RVC4_THRESHOLDS, device);
    } else if(device->getPlatform() == dai::Platform::RVC2) {
        testFsync(fps, RVC2_THRESHOLDS, device);
    } else {
        throw std::runtime_error("Unsupported platform");
    }
}

TEST_CASE("Test Fsync with different FPS values", "[fsync]") {
    // Specify a list of FPS values to test with.
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f);
    CAPTURE(fps);
    testFsyncIntegrated(fps);
}

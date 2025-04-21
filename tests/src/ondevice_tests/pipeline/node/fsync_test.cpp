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

void testFsync(float fps_mono, float fps_rgb, Thresholds thresholds, std::shared_ptr<dai::Device> device) {
    // Create pipeline
    dai::Pipeline p(std::move(device));
    auto rgb = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto sync = p.create<dai::node::Sync>();
    // Convert frame sync threshold to nanoseconds
    auto thresholdNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / std::min(fps_mono, fps_rgb)));

    sync->setSyncThreshold(thresholdNs);
    left->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, fps_mono)->link(sync->inputs["left"]);
    right->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, fps_mono)->link(sync->inputs["right"]);
    rgb->requestOutput(std::make_pair(320, 220), std::nullopt, dai::ImgResizeMode::CROP, fps_rgb)->link(sync->inputs["rgb"]);

    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(25);
    sync->out.link(benchmarkIn->input);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue(10, false);

    auto syncQueue = sync->out.createOutputQueue(10, false);
    p.start();

    for(int i = 0; i < 100; i++) {
        bool hasTimedout = false;
        auto syncData = syncQueue->get<dai::MessageGroup>(std::chrono::milliseconds(static_cast<int>(30.0 * (1000.0 / fps_mono))), hasTimedout);
        
        // Test whether pipeline stalls or not 
        if(hasTimedout) {
            FAIL("Not receiving messages for too long");
        }
        
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
        if(fps_mono == fps_rgb) {
            REQUIRE(diffAll <= thresholds.all);
        }
    }

    for(int i = 0; i < 4; i++) {
        auto reportData = benchmarkQueue->get<dai::BenchmarkReport>();
        if(i < 2) {
            // Skip the first two reports, to let the FPS stabilize.
            continue;
        }
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
#ifndef _WIN32
        // FIXME(Morato) - add back Windows once throughput is stabilized on RVC4
        REQUIRE(reportData->fps == Catch::Approx(std::min(fps_mono, fps_rgb)).epsilon(0.1));
#endif
    }
}

void testFsyncIntegrated(float fps_mono, float fps_rgb) {
    // Create device
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        testFsync(fps_mono, fps_rgb, RVC4_THRESHOLDS, device);
    } else if(device->getPlatform() == dai::Platform::RVC2) {
        testFsync(fps_mono, fps_rgb, RVC2_THRESHOLDS, device);
    } else {
        throw std::runtime_error("Unsupported platform");
    }
}

TEST_CASE("Test Fsync, mono FPS == rgb FPS", "[fsync]") {
    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 10.0f, 10.0f));
    testFsyncIntegrated(10.0f, 10.0f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 13.0f, 13.0f));
    testFsyncIntegrated(13.0f, 13.0f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 18.5f, 18.5f));
    testFsyncIntegrated(18.5f, 18.5f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 30.0f, 30.0f));
    testFsyncIntegrated(30.0f, 30.0f);
}

TEST_CASE("Test Fsync, mono FPS > rgb FPS", "[fsync]") {
    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 11.0f, 10.0f));
    testFsyncIntegrated(11.0f, 10.0f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 13.0f, 6.0f));
    testFsyncIntegrated(13.0f, 6.0f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 23.0f, 8.0f));
    testFsyncIntegrated(23.0f, 8.0f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 30.0f, 20.0f));
    testFsyncIntegrated(30.0f, 20.0f);
}

TEST_CASE("Test Fsync, mono FPS < rgb FPS", "[fsync]") {

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 10.0f, 11.0f));
    testFsyncIntegrated(10.0f, 11.0f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 6.0f, 13.0f));
    testFsyncIntegrated(6.0f, 13.0f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 8.0f, 23.0f));
    testFsyncIntegrated(8.0f, 23.0f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 20.0f, 30.0f));
    testFsyncIntegrated(20.0f, 30.0f);
}
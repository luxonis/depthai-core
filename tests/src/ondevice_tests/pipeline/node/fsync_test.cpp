#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
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
    auto thresholdNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / std::max(fps_mono, fps_rgb)));
    sync->setSyncThreshold(thresholdNs);

    // Create benchmark nodes
    auto benchmarkInLeft = p.create<dai::node::BenchmarkIn>();
    auto benchmarkInRight = p.create<dai::node::BenchmarkIn>();
    auto benchmarkInRgb = p.create<dai::node::BenchmarkIn>();
    benchmarkInLeft->sendReportEveryNMessages(150);
    benchmarkInRight->sendReportEveryNMessages(150);
    benchmarkInRgb->sendReportEveryNMessages(150);

    // Request camera outputs and set up the pipeline chain:
    // Camera -> BenchmarkIn -> Sync
    auto leftOutput = left->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, fps_mono);
    auto rightOutput = right->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, fps_mono);
    auto rgbOutput = rgb->requestOutput(std::make_pair(320, 220), std::nullopt, dai::ImgResizeMode::CROP, fps_rgb);

    leftOutput->link(benchmarkInLeft->input);
    rightOutput->link(benchmarkInRight->input);
    rgbOutput->link(benchmarkInRgb->input);

    leftOutput->link(sync->inputs["left"]);
    rightOutput->link(sync->inputs["right"]);
    rgbOutput->link(sync->inputs["rgb"]);

    // Create queues for benchmark reports
    auto benchmarkQueueLeft = benchmarkInLeft->report.createOutputQueue(10, false);
    auto benchmarkQueueRight = benchmarkInRight->report.createOutputQueue(10, false);
    auto benchmarkQueueRgb = benchmarkInRgb->report.createOutputQueue(10, false);

    auto syncQueue = sync->out.createOutputQueue(10, false);
    p.start();

    for(int i = 0; i < 100; i++) {
        bool hasTimedout = false;
        auto syncData = syncQueue->get<dai::MessageGroup>(std::chrono::seconds(2), hasTimedout);
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
        auto reportDataLeft = benchmarkQueueLeft->get<dai::BenchmarkReport>();
        auto reportDataRight = benchmarkQueueRight->get<dai::BenchmarkReport>();
        auto reportDataRgb = benchmarkQueueRgb->get<dai::BenchmarkReport>();

        if(i < 2) {
            // Skip the first two reports, to let the FPS stabilize.
            continue;
        }

        REQUIRE(reportDataLeft != nullptr);
        REQUIRE(reportDataRight != nullptr);
        REQUIRE(reportDataRgb != nullptr);

#ifndef _WIN32
        // FIXME(Morato) - add back Windows once throughput is stabilized on RVC4
        REQUIRE(reportDataLeft->fps == Catch::Approx(fps_mono).epsilon(0.1));
        REQUIRE(reportDataRight->fps == Catch::Approx(fps_mono).epsilon(0.1));
        REQUIRE(reportDataRgb->fps == Catch::Approx(fps_rgb).epsilon(0.1));
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

TEST_CASE("Test Fsync mono FPS == rgb FPS", "[fsync]") {
    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 10.0f, 10.0f));
    testFsyncIntegrated(10.0f, 10.0f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 13.0f, 13.0f));
    testFsyncIntegrated(13.0f, 13.0f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 18.5f, 18.5f));
    testFsyncIntegrated(18.5f, 18.5f);

    CAPTURE(fmt::format("== Testing with mono FPS: {}, rgb FPS: {}", 30.0f, 30.0f));
    testFsyncIntegrated(30.0f, 30.0f);
}

TEST_CASE("Test Fsync mono FPS > rgb FPS", "[fsync]") {
    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 11.0f, 10.0f));
    testFsyncIntegrated(11.0f, 10.0f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 13.0f, 6.0f));
    testFsyncIntegrated(13.0f, 6.0f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 23.0f, 8.0f));
    testFsyncIntegrated(23.8342f, 7.312f);

    CAPTURE(fmt::format("> Testing with mono FPS: {}, rgb FPS: {}", 30.0f, 20.0f));
    testFsyncIntegrated(30.0f, 20.0f);
}

TEST_CASE("Test Fsync mono FPS < rgb FPS", "[fsync]") {
    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 10.0f, 11.0f));
    testFsyncIntegrated(10.0f, 11.0f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 6.0f, 13.0f));
    testFsyncIntegrated(6.0f, 13.0f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 8.0f, 23.0f));
    testFsyncIntegrated(7.312f, 23.432f);

    CAPTURE(fmt::format("< Testing with mono FPS: {}, rgb FPS: {}", 20.0f, 30.0f));
    testFsyncIntegrated(20.0f, 30.0f);
}

TEST_CASE("Two mono-cameras with different FPS and AUTO fsync mode throws an error", "[fsync]") {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        auto p = dai::Pipeline(std::move(device));
        auto mono1 = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto mono2 = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        auto leftOutput = mono1->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, 10.0f);
        auto rightOutput = mono2->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, 20.0f);

        auto queue1 = leftOutput->createOutputQueue();
        auto queue2 = rightOutput->createOutputQueue();

        // Should throw an error because we are requesting two mono cameras
        // with different FPS and fsync mode is not explicitly specified (AUTO = INPUT)
        REQUIRE_THROWS(p.start());
    }
}

TEST_CASE("Two mono-cameras with different FPS and explicit fsync mode does not throw an error", "[fsync]") {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        auto p = dai::Pipeline(std::move(device));
        auto mono1 = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto mono2 = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        mono1->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OFF);
        mono2->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);

        auto leftOutput = mono1->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, 10.0f);
        auto rightOutput = mono2->requestOutput(std::make_pair(320, 240), std::nullopt, dai::ImgResizeMode::CROP, 20.0f);

        auto queue1 = leftOutput->createOutputQueue();
        auto queue2 = rightOutput->createOutputQueue();

        // Should not throw an error because we are explicitly setting the fsync mode to INPUT
        // for one camera and OFF for the other.
        REQUIRE_NOTHROW(p.start());
    }
}

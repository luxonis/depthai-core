#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

void testBenchmarkIn(bool benchmarkInRunOnHost, bool benchmarkOutRunOnHost, float fps, bool passthrough) {
    // Create pipeline
    dai::Pipeline p;
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(benchmarkInRunOnHost);
    auto benchmarkOut = p.create<dai::node::BenchmarkOut>();
    benchmarkOut->setRunOnHost(benchmarkOutRunOnHost);
    benchmarkOut->setFps(fps);
    benchmarkOut->out.link(benchmarkIn->input);

    auto inputQueue = benchmarkOut->input.createInputQueue();
    auto reportQueue = benchmarkIn->report.createOutputQueue();
    std::shared_ptr<dai::MessageQueue> passthroughQueue;
    if(passthrough) {
        passthroughQueue = benchmarkIn->passthrough.createOutputQueue(10, false);
    }
    p.start();
    auto inputFrame = std::make_shared<dai::ImgFrame>();
    inputQueue->send(inputFrame);
    for(int i = 0; i < 10; i++) {
        if(passthrough) {
            auto passthroughFrame = passthroughQueue->get<dai::ImgFrame>();
            REQUIRE(passthroughFrame != nullptr);
        }
        auto reportData = reportQueue->get<dai::BenchmarkReport>();
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
        if(i > 5) {
            REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(0.2));
        }
    }
}

void testCameraBenchmarking(float fps) {
    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = cam->requestOutput(std::pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps);
    REQUIRE(output != nullptr);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    output->link(benchmarkIn->input);
    auto reportQueue = benchmarkIn->report.createOutputQueue();
    p.start();
    for(int i = 0; i < 10; i++) {
        auto reportData = reportQueue->get<dai::BenchmarkReport>();
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
        if(i > 5) {
            REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(0.1));
        }
    }
}

TEST_CASE("BenchmarkIn and BenchmarkOut run on device") {
    testBenchmarkIn(false, false, 30.0f, true);
}

TEST_CASE("BenchmarkIn run on host, BenchmarkOut run on device") {
    testBenchmarkIn(true, false, 30.0f, true);
}

TEST_CASE("BenchmarkIn run on device, BenchmarkOut run on host") {
    testBenchmarkIn(false, true, 30.0f, true);
}

TEST_CASE("BenchmarkIn and BenchmarkOut run on host") {
    testBenchmarkIn(true, true, 30.0f, true);
}

TEST_CASE("BenchmarkIn and BenchmarkOut run on device - high FPS") {
    testBenchmarkIn(false, false, 1000.0f, false);
}

TEST_CASE("Camera benchmarking") {
    testCameraBenchmarking(30.0f);
}

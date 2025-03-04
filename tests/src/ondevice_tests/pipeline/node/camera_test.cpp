#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"



TEST_CASE("Test raw camera output") {
    // Create pipeline
    dai::Pipeline p;
    auto fpsToRun = GENERATE(10.5, 20.0, 30.0);
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::AUTO, std::nullopt, fpsToRun);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    SECTION("RAW"){
        camera->raw.link(benchmarkIn->input);
    }
    SECTION("NORMAL") {
        auto* output = camera->requestFullResolutionOutput();
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
    }
    benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fpsToRun) * 2));
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();

    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(fpsToRun).margin(fpsToRun * 0.1));
    }
}

TEST_CASE("Test camera with multiple outputs with different FPS") {
    // Create pipeline
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    std::map<float, std::shared_ptr<dai::MessageQueue>> fpsToReportQueue;
    bool haveRaw = false;
    std::shared_ptr<dai::MessageQueue> rawQueue;
    // for(auto fps : {10.5f, 30.0f}) { // Only one additional output in case raw is enabled is supported on RVC4
    for(auto fps : {30.0f}) {
        auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
        benchmarkIn->sendReportEveryNMessages(static_cast<uint32_t>(std::round(fps) * 2));
        auto* output = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps);
        REQUIRE(output != nullptr);
        output->link(benchmarkIn->input);
        fpsToReportQueue[fps] = benchmarkIn->report.createOutputQueue();
    }

    SECTION("With RAW") {
        auto benchmarkInRaw = p.create<dai::node::BenchmarkIn>();
        camera->raw.link(benchmarkInRaw->input);
        rawQueue = benchmarkInRaw->report.createOutputQueue();
        haveRaw = true;
    }

    SECTION("No RAW") {
        haveRaw = false;
    }

    p.start();
    for (int i = 0; i < 3; i++) {
        for(auto& [fps, queue] : fpsToReportQueue) {
            auto benchmarkReport = queue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(fps).margin(fps * 0.1));
        }
        if(haveRaw) {
            auto benchmarkReport = rawQueue->get<dai::BenchmarkReport>();
            // Allow +-10% difference
            REQUIRE(benchmarkReport->fps == Catch::Approx(30.0).margin(30.0 * 0.1));
        }
    }
}
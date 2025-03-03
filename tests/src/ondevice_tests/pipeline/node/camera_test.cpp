#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"



TEST_CASE("Test raw camera output") {
    // Create pipeline
    dai::Pipeline p;
    auto fpsToRun = GENERATE(10.5, 20.0, 30.0);
    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::AUTO, std::nullopt, fpsToRun);
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    camera->raw.link(benchmarkIn->input);
    benchmarkIn->sendReportEveryNMessages(50);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue();

    p.start();

    for(int i = 0; i < 3; i++) {
        auto benchmarkReport = benchmarkQueue->get<dai::BenchmarkReport>();
        // Allow +-10% difference
        REQUIRE(benchmarkReport->fps == Catch::Approx(fpsToRun).margin(fpsToRun * 0.1));
    }
}

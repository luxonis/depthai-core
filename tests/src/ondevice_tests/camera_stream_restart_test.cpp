#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

TEST_CASE("Test reconnecting to the pipeline multiple times") {
    constexpr auto TIMES_TO_RUN = 10;
    for(int i = 0; i < TIMES_TO_RUN; i++) {
        dai::Pipeline p;
        auto camera = p.create<dai::node::Camera>()->build();
        auto* cameraOutput = camera->requestOutput(std::make_pair(640, 400));
        REQUIRE(cameraOutput != nullptr);
        auto outputQueue = cameraOutput->createOutputQueue();
        p.start();
        // Wait for the first image
        bool hasTimedOut = false;
        auto img = outputQueue->get<dai::ImgFrame>(std::chrono::duration<double>(15), hasTimedOut);
        REQUIRE(!hasTimedOut);
        REQUIRE(img != nullptr);
    }
}

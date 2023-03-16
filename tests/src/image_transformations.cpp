// Add test case for every transformation + edge cases

#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <chrono>
#include <thread>
#include <iostream>

#include "depthai/depthai.hpp"

int testFov() {
    dai::ImgFrame frame;
    frame.setSourceSize(1280, 800);
    frame.setSourceDFov(82);
    REQUIRE_THAT(82.0, Catch::Matchers::WithinAbs(frame.getSourceDFov(), 0.1));
    REQUIRE_THAT(72.79, Catch::Matchers::WithinAbs(frame.getSourceHFov(), 0.1));
    REQUIRE_THAT(49.47, Catch::Matchers::WithinAbs(frame.getSourceVFov(), 0.1));
    return 0;
}

TEST_CASE("Test FOV") {
    testFov();
}
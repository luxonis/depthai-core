#include <catch2/catch_test_macros.hpp>
#include <memory>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionsCrop exposes and connects its subnodes", "[DetectionsCrop][onhost]") {
    auto detectionsCrop = dai::node::DetectionsCrop::create(nullptr);

    REQUIRE_FALSE(detectionsCrop->runOnHost());
    REQUIRE_FALSE(detectionsCrop->cropConfigGenerator->runOnHost());
    REQUIRE_FALSE(detectionsCrop->imageManip->runOnHost());
    REQUIRE(detectionsCrop->imageManip->inputConfig.getWaitForMessage());

    REQUIRE(&detectionsCrop->inputDetections == &detectionsCrop->cropConfigGenerator->inputDetections);
    REQUIRE(&detectionsCrop->inputImage == &detectionsCrop->cropConfigGenerator->inputImage);
    REQUIRE(&detectionsCrop->out == &detectionsCrop->imageManip->out);

    const auto configConnections = detectionsCrop->cropConfigGenerator->outConfig.getConnections();
    const auto imageConnections = detectionsCrop->cropConfigGenerator->outImage.getConnections();
    REQUIRE(configConnections.size() == 1);
    REQUIRE(imageConnections.size() == 1);
    REQUIRE(configConnections.front().in == &detectionsCrop->imageManip->inputConfig);
    REQUIRE(imageConnections.front().in == &detectionsCrop->imageManip->inputImage);
}

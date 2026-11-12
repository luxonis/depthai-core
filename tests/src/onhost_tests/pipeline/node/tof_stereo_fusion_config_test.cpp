#include <catch2/catch_all.hpp>
#include <depthai/beta/node/ToFStereoFusion.hpp>
#include <depthai/depthai.hpp>
#include <stdexcept>

using Properties = dai::beta::ToFStereoFusionProperties;

TEST_CASE("ToFStereoFusion config defaults to the calibrated threshold") {
    const Properties properties;
    REQUIRE(properties.initialConfig.confidenceThreshold == Catch::Approx(0.5f));
    REQUIRE_FALSE(properties.initialConfig.cropToOverlap);
}

TEST_CASE("ToFStereoFusion config rejects thresholds outside [0, 1]") {
    dai::beta::ToFStereoFusionConfig config;
    REQUIRE_THROWS_AS(config.setConfidenceThreshold(-0.01f), std::invalid_argument);
    REQUIRE_THROWS_AS(config.setConfidenceThreshold(1.01f), std::invalid_argument);
    REQUIRE(config.setConfidenceThreshold(0.0f).confidenceThreshold == 0.0f);
    REQUIRE(config.setConfidenceThreshold(1.0f).confidenceThreshold == 1.0f);
}

TEST_CASE("ToFStereoFusion properties survive a serialization roundtrip") {
    Properties properties;
    properties.initialConfig.setConfidenceThreshold(0.584f);

    Properties restored;
    dai::utility::deserialize(dai::utility::serialize(properties), restored);

    REQUIRE(restored.initialConfig.confidenceThreshold == Catch::Approx(0.584f));
}

#include <catch2/catch_all.hpp>
#include <depthai/beta/node/ToFStereoFusion.hpp>
#include <depthai/depthai.hpp>
#include <stdexcept>

using Properties = dai::beta::ToFStereoFusionProperties;

TEST_CASE("ToFStereoFusion properties default to the alpha-blend model") {
    const Properties properties;
    REQUIRE(properties.model == Properties::Model::ALPHA_BLEND);
    REQUIRE(properties.backend == Properties::Backend::AUTO);
    REQUIRE(properties.tofConfidenceScale == Properties::ToFConfidenceScale::FUSION);
    REQUIRE_FALSE(properties.initialConfig.nearRangePassthrough);
}

TEST_CASE("ToFStereoFusion properties survive a serialization roundtrip") {
    Properties properties;
    properties.model = Properties::Model::STEREO_NETWORK;
    properties.backend = Properties::Backend::CPU;
    properties.tofConfidenceScale = Properties::ToFConfidenceScale::VENDOR;
    properties.initialConfig.setConfidenceThreshold(0.584f).setNearRangePassthrough(true, 120.0f, 300.0f, 400.0f);

    Properties restored;
    dai::utility::deserialize(dai::utility::serialize(properties), restored);

    REQUIRE(restored.model == Properties::Model::STEREO_NETWORK);
    REQUIRE(restored.backend == Properties::Backend::CPU);
    REQUIRE(restored.tofConfidenceScale == Properties::ToFConfidenceScale::VENDOR);
    REQUIRE(restored.initialConfig.confidenceThreshold == Catch::Approx(0.584f));
    REQUIRE(restored.initialConfig.nearRangePassthrough);
    REQUIRE(restored.initialConfig.nearRangeMinDepth == Catch::Approx(120.0f));
    REQUIRE(restored.initialConfig.nearRangeTofMaxDepth == Catch::Approx(300.0f));
    REQUIRE(restored.initialConfig.nearRangeFusedMaxDepth == Catch::Approx(400.0f));
}

TEST_CASE("ToFStereoFusion config rejects invalid near-range depths") {
    dai::beta::ToFStereoFusionConfig config;
    REQUIRE_THROWS_AS(config.setNearRangePassthrough(true, 0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(config.setNearRangePassthrough(true, 400.0f, 300.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(config.setNearRangePassthrough(true, 150.0f, 343.0f, -1.0f), std::invalid_argument);
    REQUIRE_NOTHROW(config.setNearRangePassthrough(false));
}

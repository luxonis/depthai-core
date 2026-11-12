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
}

TEST_CASE("ToFStereoFusion properties survive a serialization roundtrip") {
    Properties properties;
    properties.model = Properties::Model::STEREO_NETWORK;
    properties.backend = Properties::Backend::CPU;
    properties.tofConfidenceScale = Properties::ToFConfidenceScale::VENDOR;
    properties.initialConfig.setConfidenceThreshold(0.584f);

    Properties restored;
    dai::utility::deserialize(dai::utility::serialize(properties), restored);

    REQUIRE(restored.model == Properties::Model::STEREO_NETWORK);
    REQUIRE(restored.backend == Properties::Backend::CPU);
    REQUIRE(restored.tofConfidenceScale == Properties::ToFConfidenceScale::VENDOR);
    REQUIRE(restored.initialConfig.confidenceThreshold == Catch::Approx(0.584f));
}

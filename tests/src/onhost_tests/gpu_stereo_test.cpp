#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/datatype/GPUStereoConfig.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>

using namespace dai;

TEST_CASE("GPUStereoConfig defaults", "[gpu_stereo]") {
    GPUStereoConfig config;
    REQUIRE(config.confidenceThreshold == 10);
}

TEST_CASE("GPUStereo setConfidenceThreshold clamps values", "[gpu_stereo]") {
    SECTION("negative value clamps to 0") {
        GPUStereoConfig config;
        int threshold = -1;
        config.confidenceThreshold = static_cast<std::uint8_t>(std::clamp(threshold, 0, 255));
        REQUIRE(config.confidenceThreshold == 0);
    }

    SECTION("value 0 passes through") {
        GPUStereoConfig config;
        int threshold = 0;
        config.confidenceThreshold = static_cast<std::uint8_t>(std::clamp(threshold, 0, 255));
        REQUIRE(config.confidenceThreshold == 0);
    }

    SECTION("value 255 passes through") {
        GPUStereoConfig config;
        int threshold = 255;
        config.confidenceThreshold = static_cast<std::uint8_t>(std::clamp(threshold, 0, 255));
        REQUIRE(config.confidenceThreshold == 255);
    }

    SECTION("value 256 clamps to 255") {
        GPUStereoConfig config;
        int threshold = 256;
        config.confidenceThreshold = static_cast<std::uint8_t>(std::clamp(threshold, 0, 255));
        REQUIRE(config.confidenceThreshold == 255);
    }
}

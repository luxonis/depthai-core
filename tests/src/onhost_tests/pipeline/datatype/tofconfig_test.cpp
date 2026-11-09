#include "depthai/pipeline/datatype/ToFConfig.hpp"

#include <catch2/catch_all.hpp>
#include <cstdint>
#include <vector>

#include "depthai/properties/ToFProperties.hpp"
#include "depthai/utility/Serialization.hpp"
TEST_CASE("ToF depth mode defaults to STANDARD and survives profile selection", "[ToFConfig]") {
    dai::ToFConfig config;
    REQUIRE(config.depthMode == dai::ToFConfig::DepthMode::STANDARD);
    REQUIRE(dai::ToFProperties{}.initialConfig.depthMode == dai::ToFConfig::DepthMode::STANDARD);

    REQUIRE(&config.setDepthMode(dai::ToFConfig::DepthMode::FUSION) == &config);
    for(auto profile : {dai::ToFConfig::Profile::LOW_RANGE, dai::ToFConfig::Profile::MID_RANGE, dai::ToFConfig::Profile::HIGH_RANGE}) {
        config.setProfilePreset(profile);
        REQUIRE(config.depthMode == dai::ToFConfig::DepthMode::FUSION);
    }
    config.setDepthMode(dai::ToFConfig::DepthMode::STANDARD);
    REQUIRE(config.depthMode == dai::ToFConfig::DepthMode::STANDARD);
}

TEST_CASE("ToF depth mode reaches initial and runtime configurations", "[ToFConfig]") {
    for(auto mode : {dai::ToFConfig::DepthMode::STANDARD, dai::ToFConfig::DepthMode::FUSION}) {
        dai::ToFProperties properties;
        properties.initialConfig.setDepthMode(mode);
        properties.initialConfig.phaseUnwrappingLevel = 2;
        properties.initialConfig.setMedianFilter(dai::filters::params::MedianFilter::KERNEL_5x5);
        const auto initialBytes = dai::utility::serialize(properties);
        dai::ToFProperties restored;
        REQUIRE(dai::utility::deserialize(initialBytes, restored));
        REQUIRE(restored.initialConfig.depthMode == mode);
        REQUIRE(restored.initialConfig.phaseUnwrappingLevel == 2);
        REQUIRE(restored.initialConfig.median == dai::filters::params::MedianFilter::KERNEL_5x5);

        std::vector<std::uint8_t> metadata;
        dai::DatatypeEnum datatype;
        properties.initialConfig.serialize(metadata, datatype);
        REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
        dai::ToFConfig runtimeConfig;
        REQUIRE(dai::utility::deserialize(metadata, runtimeConfig));
        REQUIRE(runtimeConfig.depthMode == mode);

        nlohmann::json json = runtimeConfig;
        REQUIRE(json.at("depthMode").get<dai::ToFConfig::DepthMode>() == mode);
        REQUIRE(json.get<dai::ToFConfig>().depthMode == mode);
    }
}

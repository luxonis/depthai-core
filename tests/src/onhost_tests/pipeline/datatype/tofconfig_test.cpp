#include <catch2/catch_all.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/properties/ToFProperties.hpp"
#include "depthai/utility/Serialization.hpp"

TEST_CASE("Intermediate ToF depth is opt-in and survives profile selection", "[ToFConfig]") {
    dai::ToFConfig config;
    REQUIRE_FALSE(config.enableIntermediateDepth);
    config.setEnableIntermediateDepth(true);
    for(auto profile : {dai::ToFConfig::Profile::LOW_RANGE, dai::ToFConfig::Profile::MID_RANGE, dai::ToFConfig::Profile::HIGH_RANGE}) {
        config.setProfilePreset(profile);
        REQUIRE(config.enableIntermediateDepth);
    }
    config.setEnableIntermediateDepth(false);
    REQUIRE_FALSE(config.enableIntermediateDepth);
}

TEST_CASE("Intermediate ToF depth selection reaches initial and runtime configurations", "[ToFConfig]") {
    for(bool enable : {false, true}) {
        for(bool burst : {false, true}) {
            dai::ToFProperties properties;
            properties.initialConfig.setEnableIntermediateDepth(enable);
            properties.initialConfig.enableBurstMode = burst;
            properties.initialConfig.enableFPPNCorrection = true;
            properties.initialConfig.enablePhaseUnwrapping = true;
            properties.initialConfig.setMedianFilter(dai::filters::params::MedianFilter::KERNEL_7x7);
            const auto initialBytes = dai::utility::serialize(properties);
            dai::ToFProperties restored;
            REQUIRE(dai::utility::deserialize(initialBytes, restored));
            REQUIRE(restored.initialConfig.enableIntermediateDepth == enable);
            REQUIRE(restored.initialConfig.enableBurstMode == burst);
            REQUIRE(restored.initialConfig.enableFPPNCorrection.value());
            REQUIRE(restored.initialConfig.enablePhaseUnwrapping.value());
            REQUIRE(restored.initialConfig.median == dai::filters::params::MedianFilter::KERNEL_7x7);

            std::vector<std::uint8_t> metadata;
            dai::DatatypeEnum datatype;
            properties.initialConfig.serialize(metadata, datatype);
            REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
            dai::ToFConfig runtimeConfig;
            REQUIRE(dai::utility::deserialize(metadata, runtimeConfig));
            REQUIRE(runtimeConfig.enableIntermediateDepth == enable);
            REQUIRE(runtimeConfig.enableBurstMode == burst);
            REQUIRE(runtimeConfig.enableFPPNCorrection.value());
            REQUIRE(runtimeConfig.enablePhaseUnwrapping.value());

            nlohmann::json json = runtimeConfig;
            REQUIRE(json.at("enableIntermediateDepth").get<bool>() == enable);
            REQUIRE(json.get<dai::ToFConfig>().enableIntermediateDepth == enable);
        }
    }
}

#include <catch2/catch_test_macros.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

TEST_CASE("ToF standard presets populate VD55H1 controls independently of phase shuffle") {
    dai::ToFConfig config;
    REQUIRE(config.phaseUnwrapErrorThreshold == 100);
    REQUIRE(config.profile == dai::ToFConfig::Profile::MID_RANGE);
    REQUIRE(config.vd55h1.phaseUnwrapErrorThreshold == 192.0f);
    REQUIRE(config.vd55h1.enableTemporalNoiseReduction == true);

    config.phaseUnwrapErrorThreshold = 42;
    for(bool phaseShuffle : {true, false}) {
        config.enablePhaseShuffleTemporalFilter = phaseShuffle;
        for(auto profile : {dai::ToFConfig::Profile::LOW_RANGE, dai::ToFConfig::Profile::MID_RANGE, dai::ToFConfig::Profile::HIGH_RANGE}) {
            config.setProfilePreset(profile);
            const bool low = profile == dai::ToFConfig::Profile::LOW_RANGE;
            const bool high = profile == dai::ToFConfig::Profile::HIGH_RANGE;
            const auto& params = config.vd55h1;
            REQUIRE(config.profile == profile);
            REQUIRE(config.enablePhaseShuffleTemporalFilter == phaseShuffle);
            REQUIRE(config.phaseUnwrapErrorThreshold == (low ? 50 : high ? 130 : 75));
            REQUIRE(params.phaseUnwrapErrorThreshold == (low ? 82.0f : high ? 300.0f : 192.0f));
            REQUIRE(params.enableBilateralFilter == true);
            REQUIRE(params.bilateralStdFactor == (low ? 7.266f : 2.051f));
            REQUIRE(params.bilateralKernelSize == 5);
            REQUIRE(params.enableTemporalNoiseReduction == true);
            REQUIRE(params.temporalNoiseReductionMaxGain == (low ? 1 : 27));
            REQUIRE(params.temporalNoiseReductionStdFactor == (low ? 0.9039f : 0.8205f));
            REQUIRE(params.enableFlyingPixelFilter == true);
            REQUIRE(params.flyingPixelDepthThreshold == (low ? 191.3f : 100.9f));
            REQUIRE(params.flyingPixelMinDepthOccurrence == (low ? 14.95f : 13.56f));
        }
    }
}

TEST_CASE("ToF config serialization preserves presets and optional controls") {
    dai::ToFConfig config;
    SECTION("Preset") {
        config.setProfilePreset(dai::ToFConfig::Profile::LOW_RANGE);
    }
    SECTION("Unset controls") {
        config.vd55h1 = {};
    }
    SECTION("Partial update with false and zero") {
        config.vd55h1 = {};
        config.vd55h1.enableBilateralFilter = false;
        config.vd55h1.phaseUnwrapErrorThreshold = 0.0f;
        config.vd55h1.temporalNoiseReductionMaxGain = 0;
        config.vd55h1.flyingPixelMinDepthOccurrence = 12.5f;
    }

    // Exercise the public message serializer, not just the nested controls.
    config.phaseUnwrapErrorThreshold = 42;
    config.enablePhaseShuffleTemporalFilter = false;
    config.enableFPPNCorrection = false;
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype{};
    config.serialize(metadata, datatype);
    REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
    dai::ToFConfig decoded;
    REQUIRE(dai::utility::deserialize(metadata, decoded));
    REQUIRE(nlohmann::json(decoded) == nlohmann::json(config));
}

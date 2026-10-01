#include <catch2/catch_test_macros.hpp>
#include <type_traits>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

TEST_CASE("ToF S5K63D aliases S5K33D controls") {
    STATIC_REQUIRE(std::is_same_v<dai::ToFConfig::S5K33D, dai::ToFConfig::S5K63D>);
    dai::ToFConfig config;
    REQUIRE(&config.s5k63d() == &config.s5k33d);
    config.s5k63d().phaseUnwrapErrorThreshold = 42;
    REQUIRE(config.s5k33d.phaseUnwrapErrorThreshold == 42);

    auto copy = config;
    copy.s5k63d().phaseUnwrapErrorThreshold = 50;
    REQUIRE(config.s5k33d.phaseUnwrapErrorThreshold == 42);
    const auto& constCopy = copy;
    REQUIRE(&constCopy.s5k63d() == &constCopy.s5k33d);
    REQUIRE(constCopy.s5k63d().phaseUnwrapErrorThreshold == 50);
    const nlohmann::json json = config;
    REQUIRE(json.at("s5k33d").at("phaseUnwrapErrorThreshold") == 42);
    REQUIRE_FALSE(json.contains("s5k63d"));
}

TEST_CASE("ToF standard presets populate VD55H1 controls independently of phase shuffle") {
    dai::ToFConfig config;
    REQUIRE(config.s5k33d.phaseUnwrapErrorThreshold == 100);
    REQUIRE(config.s5k33d.phaseUnwrappingLevel == 4);
    REQUIRE(config.s5k33d.enablePhaseShuffleTemporalFilter);
    REQUIRE_FALSE(config.s5k33d.enableBurstMode);
    REQUIRE_FALSE(config.s5k33d.enableFPPNCorrection.has_value());
    REQUIRE_FALSE(config.s5k33d.enableOpticalCorrection.has_value());
    REQUIRE_FALSE(config.s5k33d.enableTemperatureCorrection.has_value());
    REQUIRE_FALSE(config.s5k33d.enableWiggleCorrection.has_value());
    REQUIRE_FALSE(config.s5k33d.enablePhaseUnwrapping.has_value());
    REQUIRE(config.profile == dai::ToFConfig::Profile::MID_RANGE);
    REQUIRE(config.vd55h1.phaseUnwrapErrorThreshold == 192.0f);
    REQUIRE(config.vd55h1.enableTemporalNoiseReduction == true);

    config.s5k33d.phaseUnwrapErrorThreshold = 42;
    for(bool phaseShuffle : {true, false}) {
        config.s5k33d.enablePhaseShuffleTemporalFilter = phaseShuffle;
        for(auto profile : {dai::ToFConfig::Profile::LOW_RANGE, dai::ToFConfig::Profile::MID_RANGE, dai::ToFConfig::Profile::HIGH_RANGE}) {
            config.setProfilePreset(profile);
            const bool low = profile == dai::ToFConfig::Profile::LOW_RANGE;
            const bool high = profile == dai::ToFConfig::Profile::HIGH_RANGE;
            const auto& params = config.vd55h1;
            REQUIRE(config.profile == profile);
            REQUIRE(config.s5k33d.enablePhaseShuffleTemporalFilter == phaseShuffle);
            REQUIRE(config.s5k33d.phaseUnwrapErrorThreshold == (low ? 50 : high ? 130 : 75));
            REQUIRE(params.phaseUnwrapErrorThreshold == (low ? 82.0f : high ? 300.0f : 192.0f));
            REQUIRE(params.enableBilateralFilter == true);
            REQUIRE(params.bilateralStdFactor == (low ? 7.266f : 2.051f));
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
    SECTION("S5K33D controls") {
        config.s5k33d.phaseUnwrappingLevel = 0;
        config.s5k33d.enableBurstMode = true;
        config.s5k33d.enableOpticalCorrection = false;
        config.s5k33d.enableTemperatureCorrection = true;
        config.s5k33d.enableWiggleCorrection = false;
        config.s5k33d.enablePhaseUnwrapping = false;
    }

    // Exercise the public message serializer, not just the nested controls.
    config.s5k33d.phaseUnwrapErrorThreshold = 42;
    config.s5k33d.enablePhaseShuffleTemporalFilter = false;
    config.s5k33d.enableFPPNCorrection = false;
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype{};
    config.serialize(metadata, datatype);
    REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
    dai::ToFConfig decoded;
    REQUIRE(dai::utility::deserialize(metadata, decoded));
    REQUIRE(nlohmann::json(decoded) == nlohmann::json(config));
}

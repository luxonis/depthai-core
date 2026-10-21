#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <catch2/catch_test_macros.hpp>
#include <sstream>
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
    REQUIRE_FALSE(nlohmann::json(config.s5k33d).contains("enableFPPNCorrection"));
    REQUIRE_FALSE(nlohmann::json(config.s5k33d).contains("enableOpticalCorrection"));
    REQUIRE_FALSE(nlohmann::json(config.s5k33d).contains("enableTemperatureCorrection"));
    REQUIRE_FALSE(nlohmann::json(config.s5k33d).contains("enableWiggleCorrection"));
    REQUIRE_FALSE(nlohmann::json(config.s5k33d).contains("enablePhaseUnwrapping"));
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
    }

    // Exercise the public message serializer, not just the nested controls.
    config.s5k33d.phaseUnwrapErrorThreshold = 42;
    config.s5k33d.enablePhaseShuffleTemporalFilter = false;
    const nlohmann::json original = config;
    auto expected = config;
    expected.applyLegacyConfig();
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype{};
    config.serialize(metadata, datatype);
    REQUIRE(datatype == dai::DatatypeEnum::ToFConfig);
    dai::ToFConfig decoded;
    REQUIRE(dai::utility::deserialize(metadata, decoded));
    REQUIRE(nlohmann::json(decoded) == nlohmann::json(expected));
    REQUIRE(nlohmann::json(config) == original);
}

TEST_CASE("ToF legacy RVC2 fields warn and preserve supported settings") {
    std::ostringstream warnings;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(warnings);
    auto logger = std::make_shared<spdlog::logger>("tof-legacy-test", sink);
    logger->set_pattern("%v");
    logger->set_level(spdlog::level::warn);
    struct RestoreLogger {
        std::shared_ptr<spdlog::logger> previous = spdlog::default_logger();
        ~RestoreLogger() {
            spdlog::set_default_logger(previous);
        }
    } restoreLogger;
    spdlog::set_default_logger(logger);

    const nlohmann::json legacyValues = {
        {"phaseUnwrappingLevel", 0},
        {"phaseUnwrapErrorThreshold", 0},
        {"enablePhaseShuffleTemporalFilter", false},
        {"enableBurstMode", true},
        {"enableFPPNCorrection", false},
        {"enableOpticalCorrection", false},
        {"enableTemperatureCorrection", false},
        {"enableWiggleCorrection", false},
        {"enablePhaseUnwrapping", false},
    };
    dai::ToFConfig defaults;
    const nlohmann::json defaultJson = defaults;
    for(const auto& field : legacyValues.items()) {
        if(defaultJson.at("s5k33d").contains(field.key())) {
            REQUIRE(defaultJson.at(field.key()) == defaultJson.at("s5k33d").at(field.key()));
        } else {
            REQUIRE(defaultJson.at(field.key()).is_null());
        }
    }
    defaults.applyLegacyConfig();
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype{};
    defaults.serialize(metadata, datatype);
    REQUIRE(warnings.str().empty());

    for(const auto& field : legacyValues.items()) {
        CAPTURE(field.key());
        auto json = defaultJson;
        json[field.key()] = field.value();
        auto config = json.get<dai::ToFConfig>();
        const bool moved = json.at("s5k33d").contains(field.key());
        const auto expectedWarning = moved ? "use ToFConfig.s5k33d." + field.key() : "this control was removed";

        // Startup uses applyLegacyConfig; runtime messages use serialize.
        warnings.str("");
        auto initialConfig = config;
        initialConfig.applyLegacyConfig();
        REQUIRE(warnings.str().find("ToFConfig." + field.key() + " is deprecated") != std::string::npos);
        REQUIRE(warnings.str().find(expectedWarning) != std::string::npos);

        warnings.str("");
        config.serialize(metadata, datatype);
        REQUIRE(warnings.str().find(expectedWarning) != std::string::npos);
        dai::ToFConfig decoded;
        REQUIRE(dai::utility::deserialize(metadata, decoded));
        const nlohmann::json decodedJson = decoded;
        REQUIRE(decodedJson == nlohmann::json(initialConfig));
        REQUIRE(decodedJson.at(field.key()) == field.value());
        if(moved) {
            REQUIRE(decodedJson.at("s5k33d").at(field.key()) == field.value());
        } else {
            REQUIRE(decodedJson.at("s5k33d") == defaultJson.at("s5k33d"));
        }
        // Serializing must not mutate the user's nested controls.
        REQUIRE(nlohmann::json(config) == json);
    }
}

TEST_CASE("ToF decoded controls can be edited through nested or legacy fields") {
    for(const auto type : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
        for(const bool nestedMessage : {false, true}) {
            CAPTURE(type, nestedMessage);
            auto checkControl = [=](auto nestedMember, auto legacyMember, auto initialValue, auto updatedValue) {
                dai::ToFConfig source;
                source.s5k33d.*nestedMember = initialValue;
                source.applyLegacyConfig();
                dai::ToFConfig decoded;
                if(nestedMessage) {
                    const auto data = dai::utility::serialize(std::vector<dai::ToFConfig>{source}, type);
                    std::vector<dai::ToFConfig> configs;
                    REQUIRE(dai::utility::deserialize(data, configs, type));
                    REQUIRE(configs.size() == 1);
                    decoded = configs.front();
                } else {
                    const auto data = dai::utility::serialize(source, type);
                    REQUIRE(dai::utility::deserialize(data, decoded, type));
                }

                auto checkRoundtrip = [=](dai::ToFConfig config) {
                    std::vector<std::uint8_t> metadata;
                    dai::DatatypeEnum datatype{};
                    config.serialize(metadata, datatype);
                    dai::ToFConfig result;
                    REQUIRE(dai::utility::deserialize(metadata, result));
                    REQUIRE(result.s5k33d.*nestedMember == updatedValue);
                    REQUIRE(result.*legacyMember == updatedValue);
                    // Startup normalization must agree with runtime serialization.
                    config.applyLegacyConfig();
                    REQUIRE(nlohmann::json(config) == nlohmann::json(result));
                };

                auto nestedEdit = decoded;
                nestedEdit.s5k33d.*nestedMember = updatedValue;
                checkRoundtrip(nestedEdit);
                auto legacyEdit = decoded;
                legacyEdit.*legacyMember = updatedValue;
                checkRoundtrip(legacyEdit);
            };
            checkControl(&dai::ToFConfig::S5K33D::phaseUnwrappingLevel, &dai::ToFConfig::phaseUnwrappingLevel, 3, 2);
            checkControl(&dai::ToFConfig::S5K33D::phaseUnwrapErrorThreshold, &dai::ToFConfig::phaseUnwrapErrorThreshold, std::uint16_t{42}, std::uint16_t{43});
            checkControl(&dai::ToFConfig::S5K33D::enablePhaseShuffleTemporalFilter, &dai::ToFConfig::enablePhaseShuffleTemporalFilter, false, true);
            checkControl(&dai::ToFConfig::S5K33D::enableBurstMode, &dai::ToFConfig::enableBurstMode, true, false);
        }
    }
}

TEST_CASE("ToF JSON preserves pending nested edits and legacy override precedence") {
    dai::ToFConfig source;
    SECTION("Nested edit") {
        source.s5k33d.phaseUnwrapErrorThreshold = 42;
    }
    SECTION("Legacy edit") {
        source.phaseUnwrapErrorThreshold = 42;
    }
    SECTION("Legacy wins when both representations are edited") {
        source.s5k33d.phaseUnwrapErrorThreshold = 41;
        source.phaseUnwrapErrorThreshold = 42;
    }
    const nlohmann::json json = source;
    auto decoded = json.get<dai::ToFConfig>();
    decoded.applyLegacyConfig();
    REQUIRE(decoded.s5k33d.phaseUnwrapErrorThreshold == 42);
    REQUIRE(decoded.phaseUnwrapErrorThreshold == 42);
}

#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include <algorithm>

#include "depthai/common/ToFPreset.hpp"
#include "depthai/common/ToFSensorMode.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/ToF.hpp"

TEST_CASE("ToFSensorMode context frame counts", "[ToF][Host]") {
    REQUIRE(dai::getToFSensorModeContextFrames(dai::ToFSensorMode::F1_FULL) == 1);
    REQUIRE(dai::getToFSensorModeContextFrames(dai::ToFSensorMode::F2_FULL) == 2);
    REQUIRE(dai::getToFSensorModeContextFrames(dai::ToFSensorMode::F3_FULL) == 3);
    REQUIRE(dai::getToFSensorModeContextFrames(dai::ToFSensorMode::F2_BINNING_2X2) == 2);
    REQUIRE(dai::getToFSensorModeContextFrames(dai::ToFSensorMode::F3_BINNING_2X2) == 3);
}

TEST_CASE("ToFSensorMode raw output size relationship", "[ToF][Host]") {
    for(const auto mode :
        {dai::ToFSensorMode::F1_FULL, dai::ToFSensorMode::F2_FULL, dai::ToFSensorMode::F3_FULL, dai::ToFSensorMode::F2_BINNING_2X2,
         dai::ToFSensorMode::F3_BINNING_2X2}) {
        const auto [rawW, rawH] = dai::getToFSensorModeRawResolution(mode);
        const auto [outW, outH] = dai::getToFSensorModeOutputResolution(mode);
        REQUIRE(rawW > outW);
        REQUIRE(rawH > outH);
    }
}

TEST_CASE("ToFDecoderConfig round-trip from ToFConfig", "[ToF][Host]") {
    dai::ToFConfig config;
    config.median = dai::filters::params::MedianFilter::KERNEL_3x3;
    config.phaseUnwrappingLevel = 2;
    config.phaseUnwrapErrorThreshold = 42;
    config.enableBurstMode = true;
    config.enableDistortionCorrection = false;

    const auto decoder = dai::ToFDecoderConfig::fromToFConfig(config);
    dai::ToFConfig restored;
    decoder.applyTo(restored);

    REQUIRE(restored.median == config.median);
    REQUIRE(restored.phaseUnwrappingLevel == config.phaseUnwrappingLevel);
    REQUIRE(restored.phaseUnwrapErrorThreshold == config.phaseUnwrapErrorThreshold);
    REQUIRE(restored.enableBurstMode == config.enableBurstMode);
    REQUIRE(restored.enableDistortionCorrection == config.enableDistortionCorrection);
}

TEST_CASE("ToFIppConfig round-trip from ToFConfig", "[ToF][Host]") {
    dai::ToFConfig config;
    config.phaseUnwrapErrorThreshold = 88;
    config.enableBilateralFilter = true;
    config.bilateralStdFactor = 1.25f;
    config.enableFlyingPixelCorrection = false;

    const auto ipp = dai::ToFIppConfig::fromToFConfig(config);
    dai::ToFConfig restored;
    ipp.applyTo(restored);

    REQUIRE(restored.phaseUnwrapErrorThreshold == 88);
    REQUIRE(restored.enableBilateralFilter.has_value());
    REQUIRE(*restored.enableBilateralFilter == true);
    REQUIRE(restored.bilateralStdFactor.has_value());
    REQUIRE(*restored.bilateralStdFactor == Catch::Approx(1.25f));
    REQUIRE(restored.enableFlyingPixelCorrection.has_value());
    REQUIRE(*restored.enableFlyingPixelCorrection == false);
}

TEST_CASE("ToFBuildOptions defaults", "[ToF][Host]") {
    dai::ToFBuildOptions options;
    REQUIRE(options.boardSocket == dai::CameraBoardSocket::AUTO);
    REQUIRE_FALSE(options.fps.has_value());
    REQUIRE_FALSE(options.preset.has_value());
    // BETA: capture is fixed to F3_FULL — ToFBuildOptions has no sensorMode field.
}

TEST_CASE("ToFPreset maps to expected phase unwrap thresholds via ToFIppConfig", "[ToF][Host]") {
    const std::pair<dai::ToFPreset, uint16_t> expected[] = {
        {dai::ToFPreset::LOW_RANGE, 50},
        {dai::ToFPreset::MID_RANGE, 75},
        {dai::ToFPreset::HIGH_RANGE, 130},
        {dai::ToFPreset::FAST_OBJECTS, 75},
        {dai::ToFPreset::OFF, 10000},
    };

    for(const auto& [preset, threshold] : expected) {
        dai::ToFIppConfig ipp;
        ipp.applyPreset(preset);
        REQUIRE(ipp.phaseUnwrapErrorThreshold == threshold);
    }
}

TEST_CASE("ToFPreset FAST_OBJECTS disables temporal noise reduction", "[ToF][Host]") {
    dai::ToFIppConfig ipp;
    ipp.applyPreset(dai::ToFPreset::FAST_OBJECTS);
    // FAST_OBJECTS turns TNR off to reduce motion blur on fast-moving scenes.
    REQUIRE(ipp.enableTemporalNoiseReduction.has_value());
    REQUIRE(*ipp.enableTemporalNoiseReduction == false);

    // Range presets leave TNR at the IPP default (unset).
    dai::ToFIppConfig mid;
    mid.applyPreset(dai::ToFPreset::MID_RANGE);
    REQUIRE_FALSE(mid.enableTemporalNoiseReduction.has_value());
}

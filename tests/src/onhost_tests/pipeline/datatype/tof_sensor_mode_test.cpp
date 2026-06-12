#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include <depthai/common/ToFSensorMode.hpp>
#include <depthai/pipeline/datatype/ToFConfig.hpp>

TEST_CASE("ToFSensorMode raw resolution", "[ToFSensorMode]") {
    REQUIRE(dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F1_FULL) == std::pair<uint32_t, uint32_t>{1344, 2420});
    REQUIRE(dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F2_FULL) == std::pair<uint32_t, uint32_t>{1344, 4832});
    REQUIRE(dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F3_FULL) == std::pair<uint32_t, uint32_t>{1344, 7244});
    REQUIRE(dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F2_BINNING_2X2) == std::pair<uint32_t, uint32_t>{672, 2420});
    REQUIRE(dai::getToFSensorModeRawResolution(dai::ToFSensorMode::F3_BINNING_2X2) == std::pair<uint32_t, uint32_t>{672, 3626});
}

TEST_CASE("ToFSensorMode output resolution", "[ToFSensorMode]") {
    REQUIRE(dai::getToFSensorModeOutputResolution(dai::ToFSensorMode::F3_FULL) == std::pair<uint32_t, uint32_t>{804, 672});
    REQUIRE(dai::getToFSensorModeOutputResolution(dai::ToFSensorMode::F3_BINNING_2X2) == std::pair<uint32_t, uint32_t>{402, 336});
}

TEST_CASE("ToFIppConfig preset applies phase unwrap threshold", "[ToFIppConfig]") {
    dai::ToFConfig config;
    config.phaseUnwrapErrorThreshold = 999;

    dai::ToFIppConfig ipp;
    ipp.applyPreset(dai::ToFPreset::MID_RANGE);
    ipp.applyTo(config);

    REQUIRE(config.phaseUnwrapErrorThreshold == 75);
}

TEST_CASE("ToFIppConfig OFF disables IPP filters", "[ToFIppConfig]") {
    dai::ToFConfig config;
    config.enableBilateralFilter = true;

    dai::ToFIppConfig ipp;
    ipp.applyPreset(dai::ToFPreset::OFF);
    ipp.applyTo(config);

    REQUIRE(config.phaseUnwrapErrorThreshold == 10000);
    REQUIRE(config.enableBilateralFilter.has_value());
    REQUIRE(*config.enableBilateralFilter == false);
}

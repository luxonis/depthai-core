#include <catch2/catch_all.hpp>

#include "depthai/device/BoardConfig.hpp"

TEST_CASE("Production board configuration survives firmware serialization", "[BoardConfig]") {
    dai::BoardConfig config;
    config.network.forceEnable = true;
    config.network.mtu = 1400;
    config.emmc = true;
    config.eepromData.emplace();
    config.eepromData->boardName = "test-board";
    config.eepromData->boardOptions = 42;
    config.eepromData->cameraData[dai::CameraBoardSocket::CAM_A].width = 1920;

    dai::BoardConfig decoded;
    REQUIRE(dai::utility::deserialize(dai::utility::serialize(config), decoded));
    REQUIRE(decoded.network.forceEnable);
    REQUIRE(decoded.network.mtu == 1400);
    REQUIRE(decoded.emmc.value_or(false));
    REQUIRE(decoded.eepromData.has_value());
    REQUIRE(decoded.eepromData->version == config.eepromData->version);
    REQUIRE(decoded.eepromData->boardName == "test-board");
    REQUIRE(decoded.eepromData->boardOptions == 42);
    REQUIRE(decoded.eepromData->cameraData.at(dai::CameraBoardSocket::CAM_A).width == 1920);
}

TEST_CASE("Default board configuration keeps production overrides disabled", "[BoardConfig]") {
    const dai::BoardConfig config;
    dai::BoardConfig decoded;
    REQUIRE(dai::utility::deserialize(dai::utility::serialize(config), decoded));
    REQUIRE_FALSE(decoded.network.forceEnable);
    REQUIRE_FALSE(decoded.eepromData.has_value());
    REQUIRE_FALSE(decoded.emmc.has_value());
}

TEST_CASE("Network configuration preserves existing aggregate initialization", "[BoardConfig]") {
    const dai::BoardConfig::Network config{1400, false};
    REQUIRE(config.mtu == 1400);
    REQUIRE_FALSE(config.xlinkTcpNoDelay);
    REQUIRE_FALSE(config.forceEnable);
}

#include <algorithm>
#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("Forced Ethernet initialization preserves an RVC2 network boot", "[BoardConfig][poe]") {
    const auto devices = dai::Device::getAllAvailableDevices();
    const auto selected =
        std::find_if(devices.begin(), devices.end(), [](const auto& info) { return info.platform == X_LINK_MYRIAD_X && info.protocol == X_LINK_TCP_IP; });
    REQUIRE(selected != devices.end());

    dai::Device::Config config;
    config.board.network.forceEnable = true;
    dai::Device device(config, *selected);
    REQUIRE(device.getProtocol() == X_LINK_TCP_IP);
    REQUIRE(device.getEthernetLinkSpeed() > 0);
    const auto duplex = device.getEthernetLinkDuplex();
    REQUIRE((duplex == 0 || duplex == 1));
}

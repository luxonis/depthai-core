#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("Forced Ethernet initialization preserves an RVC2 network boot", "[BoardConfig][poe]") {
    dai::Device::Config config;
    config.board.network.forceEnable = true;
    // Search with the Device timeout: a POE device closed by the previous test needs a few seconds to reappear.
    dai::Device device(config);
    REQUIRE(device.getPlatform() == dai::Platform::RVC2);
    REQUIRE(device.getProtocol() == X_LINK_TCP_IP);
    REQUIRE(device.getEthernetLinkSpeed() > 0);
    const auto duplex = device.getEthernetLinkDuplex();
    REQUIRE((duplex == 0 || duplex == 1));
}

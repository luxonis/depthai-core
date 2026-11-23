#include <catch2/catch_all.hpp>
#include "depthai/depthai.hpp"
#include "utility/DeviceOSVersion.hpp"

TEST_CASE("Device OS version cache survives close without RPC", "[DeviceOSVersionLive]") {
    dai::Device device;
    const auto cached = device.getDeviceOSVersion();
    if(device.getPlatform() == dai::Platform::RVC4) {
        REQUIRE(cached == dai::utility::parseDeviceOSVersion(device.getOSVersion()));
    } else {
        REQUIRE_FALSE(cached);
    }
    device.close();
    for(int i = 0; i < 10; ++i) REQUIRE(device.getDeviceOSVersion() == cached);
}

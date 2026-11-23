#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

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

TEST_CASE("Device OS version cache refreshes after reconnect", "[DeviceOSVersionLive][Reconnect]") {
    std::atomic<bool> reconnected{false};
    dai::Device device;
    if(device.getPlatform() != dai::Platform::RVC4) SKIP("Requires RVC4");
    // Reconnection discovers by ID. A device reachable only by explicit IP cannot
    // exercise that path on hosts outside its discovery network.
    if(!std::get<0>(dai::XLinkConnection::getDeviceById(device.getDeviceId(), X_LINK_ANY_STATE, false))) {
        SKIP("Reconnect requires device discovery by ID");
    }
    device.setMaxReconnectionAttempts(3, [&](dai::DeviceBase::ReconnectionStatus status) {
        if(status == dai::DeviceBase::ReconnectionStatus::RECONNECTED) reconnected = true;
    });
    device.getConnection()->close();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
    while(!reconnected && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(reconnected.load());
    REQUIRE(device.getDeviceOSVersion() == dai::utility::parseDeviceOSVersion(device.getOSVersion()));
}

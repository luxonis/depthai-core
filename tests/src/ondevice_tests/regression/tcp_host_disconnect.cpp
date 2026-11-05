#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "subprocess.hpp"

TEST_CASE("RVC2 TCP device recovers after abrupt host exit") {
    using Clock = std::chrono::steady_clock;
    const auto devices = dai::Device::getAllAvailableDevices();
    const auto selected = std::find_if(
        devices.begin(), devices.end(), [](const dai::DeviceInfo& info) { return info.protocol == X_LINK_TCP_IP && info.platform == X_LINK_MYRIAD_X; });
    REQUIRE(selected != devices.end());
    const auto id = selected->getDeviceId();
    CAPTURE(id);

    subprocess::Popen child(std::vector<std::string>{DEPTHAI_TCP_DISCONNECT_TEST_CHILD_PATH, selected->name});
    const auto startupDeadline = Clock::now() + std::chrono::seconds(60);
    int status = child.poll();
    while(status == -1 && Clock::now() < startupDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        status = child.poll();
    }
    if(status == -1) {
        child.kill();
        child.wait();
        FAIL("Device owner did not exit within 60 seconds");
    }
    REQUIRE(status == 0);

    // System logs used to keep the watchdog alive for ~64 seconds after host exit.
    const auto exited = Clock::now();
    const auto deadline = exited + std::chrono::seconds(30);
    bool recovered = false;
    dai::DeviceInfo recoveredInfo;
    while(Clock::now() < deadline) {
        try {
            for(const auto& info : dai::Device::getAllConnectedDevices()) {
                if(info.getDeviceId() == id && info.state == X_LINK_BOOTLOADER) {
                    recoveredInfo = info;
                    recovered = true;
                    break;
                }
            }
        } catch(const std::runtime_error&) {
            // Explicit-IP discovery can fail while the device resets.
        }
        if(recovered) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    const auto seconds = std::chrono::duration<double>(Clock::now() - exited).count();
    CAPTURE(seconds);
    REQUIRE(recovered);
    REQUIRE(seconds < 30.0);
    dai::Device reopened(recoveredInfo);
    REQUIRE(reopened.getDeviceInfo().getDeviceId() == id);
}

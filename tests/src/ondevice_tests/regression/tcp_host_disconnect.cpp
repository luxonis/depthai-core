#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

#include "depthai/depthai.hpp"
#include "subprocess.hpp"
#include "utility/Platform.hpp"

namespace {

using Clock = std::chrono::steady_clock;

subprocess::env_string_t envString(const std::string& value) {
#ifdef _MSC_VER
    return std::wstring(value.begin(), value.end());
#else
    return value;
#endif
}

// Also kill/reap the owner if an assertion fails before the intentional kill.
struct Owner {
    explicit Owner(const dai::DeviceInfo& info)
        : directory(dai::platform::getTempPath()),
          process({DEPTHAI_TCP_DISCONNECT_TEST_PATH, "[.tcp-owner]"},
                  subprocess::environment{{{envString("DEPTHAI_TCP_TEST_ID"), envString(info.getDeviceId())},
                                           {envString("DEPTHAI_TCP_TEST_NAME"), envString(info.name)},
                                           {envString("DEPTHAI_TCP_TEST_READY"), envString((directory / "ready").string())}}}) {}
    ~Owner() {
        if(process.poll() == -1) process.kill();
        process.wait();
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
    std::filesystem::path directory;
    subprocess::Popen process;
};

}  // namespace

TEST_CASE("TCP device owner subprocess", "[.tcp-owner]") {
    const auto* id = std::getenv("DEPTHAI_TCP_TEST_ID");
    const auto* name = std::getenv("DEPTHAI_TCP_TEST_NAME");
    const auto* readyPath = std::getenv("DEPTHAI_TCP_TEST_READY");
    REQUIRE(id != nullptr);
    REQUIRE(readyPath != nullptr);
    REQUIRE(name != nullptr);
    dai::Device device{dai::DeviceInfo(name)};
    REQUIRE(device.getDeviceInfo().getDeviceId() == id);
    REQUIRE(device.getPlatform() == dai::Platform::RVC2);
    REQUIRE(device.getDeviceInfo().protocol == X_LINK_TCP_IP);
    device.setLogLevel(dai::LogLevel::INFO);
    device.setSystemInformationLoggingRate(1.0f);
    {
        std::ofstream ready(readyPath);
        ready << "ready\n";
        ready.close();
        REQUIRE(ready.good());
    }
    // The parent kills this process without Device::close().
    std::this_thread::sleep_for(std::chrono::minutes(2));
    FAIL("Parent did not kill the device owner");
}

TEST_CASE("RVC2 TCP device recovers promptly after host termination") {
    const auto devices = dai::Device::getAllAvailableDevices();
    const auto selected = std::find_if(
        devices.begin(), devices.end(), [](const dai::DeviceInfo& info) { return info.protocol == X_LINK_TCP_IP && info.platform == X_LINK_MYRIAD_X; });
    REQUIRE(selected != devices.end());
    const auto id = selected->getDeviceId();
    CAPTURE(id);
    Owner owner(*selected);
    const auto startupDeadline = Clock::now() + std::chrono::seconds(60);
    while(!std::filesystem::exists(owner.directory / "ready") && owner.process.poll() == -1 && Clock::now() < startupDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    REQUIRE(std::filesystem::exists(owner.directory / "ready"));
    REQUIRE(owner.process.poll() == -1);
    // Leave system logging enabled: queued writes used to keep the watchdog alive
    // for ~64 seconds even after the host disappeared.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const auto killed = Clock::now();
    owner.process.kill();
    owner.process.wait();

    const auto deadline = killed + std::chrono::seconds(30);
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
            // Explicit-IP discovery can fail transiently while the device resets.
        }
        if(recovered) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    const auto seconds = std::chrono::duration<double>(Clock::now() - killed).count();
    CAPTURE(seconds);
    REQUIRE(recovered);
    REQUIRE(seconds < 30.0);
    // Verify the recovered device can be used by the following test.
    dai::Device reopened(recoveredInfo);
    REQUIRE(reopened.getDeviceInfo().getDeviceId() == id);
}

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

constexpr auto RUN_TIMEOUT = std::chrono::minutes(1);

struct ConnectResult {
    bool success = false;
    std::string error;
};

void setEnvVar(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

std::string getDisplayName(const dai::DeviceInfo& deviceInfo) {
    if(!deviceInfo.name.empty()) {
        return deviceInfo.name;
    }
    if(!deviceInfo.getDeviceId().empty()) {
        return deviceInfo.getDeviceId();
    }
    return "<unknown>";
}

ConnectResult connectDevice(dai::DeviceInfo deviceInfo) {
    ConnectResult result;

    try {
        dai::Device device(deviceInfo);
        (void)device.getDeviceName();
        (void)device.getConnectedCameras();
        result.success = true;
    } catch(const std::exception& ex) {
        result.error = ex.what();
    }
    // std::cout << "Device: " << getDisplayName(deviceInfo) << ", Success: " << result.success << std::endl;

    return result;
}

bool allFinished(const std::vector<std::shared_future<ConnectResult>>& futures) {
    for(const auto& future : futures) {
        if(future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    setEnvVar("DEPTHAI_CONNECT_TIMEOUT", "55000");
    setEnvVar("DEPTHAI_BOOTUP_TIMEOUT", "55000");
    setEnvVar("DEPTHAI_RECONNECT_TIMEOUT", "0");

    const auto deviceInfos = dai::Device::getAllAvailableDevices();

    std::cout << "Discovered " << deviceInfos.size() << " device(s)" << std::endl;
    for(const auto& deviceInfo : deviceInfos) {
        std::cout << "  " << getDisplayName(deviceInfo) << std::endl;
    }

    if(deviceInfos.empty()) {
        return 0;
    }

    std::uint64_t runNumber = 1;
    std::uint64_t allSuccessfulRuns = 0;
    std::uint64_t runsWithTimeout = 0;

    while(true) {
        std::vector<std::shared_future<ConnectResult>> futures;
        futures.reserve(deviceInfos.size());

        for(const auto& deviceInfo : deviceInfos) {
            futures.emplace_back(std::async(std::launch::async, connectDevice, deviceInfo).share());
        }

        const auto deadline = std::chrono::steady_clock::now() + RUN_TIMEOUT;
        while(std::chrono::steady_clock::now() < deadline) {
            if(allFinished(futures)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::size_t successfulCount = 0;
        for(const auto& future : futures) {
            if(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && future.get().success) {
                successfulCount++;
            }
        }
        const auto timedOutCount = deviceInfos.size() - successfulCount;

        if(successfulCount == deviceInfos.size()) {
            allSuccessfulRuns++;
            std::cout << "Succefull: " << runNumber << std::endl;
        } else {
            runsWithTimeout++;
            std::cout << "Timed-out" << std::endl;
        }
        std::cout << "Successful: " << successfulCount << ", Timed-out: " << timedOutCount << std::endl;
        std::cout << "Global all successful: " << allSuccessfulRuns << ", Global with timeout: " << runsWithTimeout << std::endl;

        for(auto& future : futures) {
            future.wait();
        }

        runNumber++;
    }

    return 0;
}

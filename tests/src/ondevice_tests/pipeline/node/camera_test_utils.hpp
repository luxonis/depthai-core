#pragma once

// Helpers for on-device tests that sweep every connected camera and sensor config.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace camera_test {

// Covers the device dropping out of discovery while its service restarts after a connection closes
constexpr auto DEVICE_RESET_TIMEOUT = std::chrono::milliseconds(2000);

struct DeviceSnapshot {
    dai::DeviceInfo info;
    dai::Platform platform = dai::Platform::RVC4;
    std::vector<dai::CameraFeatures> cameras;
};

// Device and cameras, read once per test binary
inline const DeviceSnapshot& deviceSnapshot() {
    static const DeviceSnapshot snapshot = [] {
        dai::Device device;
        DeviceSnapshot result;
        // Only ID and platform, so the connection search retries until the device is back up after a reset
        result.info.deviceId = device.getDeviceId();
        result.info.platform = device.getDeviceInfo().platform;
        result.platform = device.getPlatform();
        result.cameras = device.getConnectedCameraFeatures();
        return result;
    }();
    return snapshot;
}

// The device restarts its service when a connection closes. Wait for it to drop out of
// discovery first, so the new connection doesn't race the restart.
inline std::shared_ptr<dai::Device> reconnect() {
    const auto& info = deviceSnapshot().info;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < DEVICE_RESET_TIMEOUT) {
        const auto devices = dai::XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, true, 50);
        const bool listed = std::any_of(devices.begin(), devices.end(), [&](const dai::DeviceInfo& device) { return device.deviceId == info.deviceId; });
        if(!listed) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return std::make_shared<dai::Device>(info);
}

// The device selects a config by resolution and HDR flag only, so a config sharing both with an
// earlier one can't be selected on its own
inline bool isSelectable(const std::vector<dai::CameraSensorConfig>& configs, size_t index) {
    const auto& config = configs[index];
    return std::none_of(configs.begin(), configs.begin() + static_cast<std::ptrdiff_t>(index), [&](const dai::CameraSensorConfig& earlier) {
        return earlier.width == config.width && earlier.height == config.height && earlier.hdr == config.hdr;
    });
}

inline std::pair<uint32_t, uint32_t> resolution(const dai::CameraSensorConfig& config) {
    return {static_cast<uint32_t>(config.width), static_cast<uint32_t>(config.height)};
}

// Camera running the given sensor config
inline std::shared_ptr<dai::node::Camera> buildCamera(dai::Pipeline& pipeline,
                                                      dai::CameraBoardSocket socket,
                                                      const dai::CameraSensorConfig& config,
                                                      float fps) {
    auto camera = pipeline.create<dai::node::Camera>();
    if(config.hdr) {
        // HDR configs share their resolution with a non-HDR config, the flag selects between them
        camera->initialControl.setHdr(true);
    }
    return camera->build(socket, resolution(config), fps);
}

}  // namespace camera_test

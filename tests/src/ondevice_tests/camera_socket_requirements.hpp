#pragma once

// Camera socket requirements of on-device test cases.
//
// Some HIL testbeds connect only some camera sockets (the RVC4 FFC pool: for example only CAM_D and CAM_E).
// The RVC4 FFC test job sets DEPTHAI_TESTS_ALLOW_MISSING_CAMERAS=1. Then a test case that needs a socket that
// is not connected is skipped. Without that variable a missing socket fails the test case, because on a
// device with a fixed camera set a missing camera is a real fault.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/DeviceBase.hpp"

namespace dai::test {

inline bool missingCamerasAllowed() {
    const char* value = std::getenv("DEPTHAI_TESTS_ALLOW_MISSING_CAMERAS");
    return value != nullptr && std::string(value) == "1";
}

inline std::string joinSocketNames(const std::vector<dai::CameraBoardSocket>& sockets) {
    if(sockets.empty()) {
        return "none";
    }
    std::string names;
    for(const auto socket : sockets) {
        if(!names.empty()) {
            names += ", ";
        }
        names += dai::toString(socket);
    }
    return names;
}

inline std::vector<dai::CameraBoardSocket> getConnectedSockets(dai::DeviceBase& device) {
    std::vector<dai::CameraBoardSocket> connected;
    for(const auto& features : device.getConnectedCameraFeatures()) {
        connected.push_back(features.socket);
    }
    return connected;
}

inline void skipOrFailMissingSockets(const std::string& missingDescription, const std::vector<dai::CameraBoardSocket>& connected) {
    const std::string message = "Camera socket not connected: " + missingDescription + " (connected: " + joinSocketNames(connected) + ")";
    if(missingCamerasAllowed()) {
        SKIP(message);
    }
    FAIL(message << ". Set DEPTHAI_TESTS_ALLOW_MISSING_CAMERAS=1 to skip this test case on a device that has only some cameras.");
}

// Skips or fails the test case unless all the given sockets are connected.
inline void requireCameraSockets(dai::DeviceBase& device, std::initializer_list<dai::CameraBoardSocket> required) {
    const auto connected = getConnectedSockets(device);
    std::vector<dai::CameraBoardSocket> missing;
    for(const auto socket : required) {
        if(std::find(connected.begin(), connected.end(), socket) == connected.end()) {
            missing.push_back(socket);
        }
    }
    if(!missing.empty()) {
        skipOrFailMissingSockets(joinSocketNames(missing), connected);
    }
}

// Skips or fails the test case unless Camera::build() with CameraBoardSocket::AUTO can find a socket:
// AUTO selects the first connected socket of CAM_A, CAM_B and CAM_C (keep this list the same as the AUTO rule in
// Camera::build(), src/pipeline/node/Camera.cpp).
inline void requireAutoCameraSocket(dai::DeviceBase& device) {
    const auto connected = getConnectedSockets(device);
    for(const auto socket : {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        if(std::find(connected.begin(), connected.end(), socket) != connected.end()) {
            return;
        }
    }
    skipOrFailMissingSockets("CAM_A, CAM_B or CAM_C (CameraBoardSocket::AUTO)", connected);
}

}  // namespace dai::test

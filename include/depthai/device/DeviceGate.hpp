#pragma once

// std
#include <cstdint>
#include <string>
#include <thread>
#include <type_traits>

// project
#include "depthai/device/Version.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

/**
 * Represents the DepthAI Gate with the methods to interact with it.
 */
class DeviceGate {
   public:
    // Static API
    /**
     * Searches for connected devices in GATE state and returns first available.
     * @returns Tuple of boolean and DeviceInfo. If found boolean is true and DeviceInfo describes the device. Otherwise false
     */
    std::tuple<bool, DeviceInfo> getFirstAvailableDevice();

    /**
     * Searches for connected devices in GATE state.
     * @returns Vector of all found devices
     */
    std::vector<DeviceInfo> getAllAvailableDevices();

    /**
     * Connects to DepthAI Gate
     * @param deviceInfo Device to connect to
     */
    DeviceGate(const DeviceInfo& deviceInfo);
    ~DeviceGate();
    bool isOkay();
    bool createSession(bool exclusive = true);
    bool startSession();

    struct VersionInfo {
        std::string gate, os;
    };
    Version getVersion();
    VersionInfo getAllVersion();

    bool isBootedNonExclusive();

   private:
    // private
    DeviceInfo deviceInfo;

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    std::string sessionId;
};

}  // namespace dai

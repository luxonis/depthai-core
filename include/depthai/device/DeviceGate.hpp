#pragma once

// std
#include <XLink/XLinkPublicDefines.h>

#include <cstdint>
#include <string>
#include <thread>
#include <type_traits>

// project
#include <optional>

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

    enum class SessionState { NOT_CREATED, CREATED, RUNNING, STOPPED, STOPPING, CRASHED, DESTROYED, ERROR_STATE };

    /**
     * Connects to DepthAI Gate
     * @param deviceInfo Device to connect to
     */
    DeviceGate(const DeviceInfo& deviceInfo);
    ~DeviceGate();
    bool isOkay();
    bool createSession(bool exclusive = true);
    bool startSession();
    bool stopSession();
    bool deleteSession();
    bool destroySession();
    SessionState getState();
    // Waits for the gate session to end and tries to get the logs and crash dump out
    void waitForSessionEnd();

    std::optional<std::vector<uint8_t>> getCrashDump(std::string& filename);

    struct VersionInfo {
        std::string gate, os;
    };
    Version getVersion();
    VersionInfo getAllVersion();

    bool isBootedNonExclusive();

   private:
    // private
    DeviceInfo deviceInfo;

    std::thread stateMonitoringThread;

    std::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl, std::string& filename);

    std::optional<std::string> saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename, std::string direcotryPath = "");

    // state of the session
    std::atomic_bool sessionCreated{false};

    XLinkPlatform_t platform;
    std::string version;
    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    std::string sessionId;
};

}  // namespace dai

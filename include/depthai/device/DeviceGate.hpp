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
#include "tl/optional.hpp"
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

    enum class SessionState {
        // Session is not created
        NOT_CREATED,
        // Session is created, but not started
        CREATED,
        // Session is started
        RUNNING,
        // Session is stopped
        STOPPED,
        // Session is stopping
        STOPPING,
        // Session is crashed
        CRASHED,
        // Session is destroyed
        DESTROYED,
        // Error
        ERROR
    };

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
    SessionState updateState();

    tl::optional<std::vector<uint8_t>> getLogFile();
    tl::optional<std::vector<uint8_t>> getCoreDump();

    struct VersionInfo {
        std::string gate, os;
    };
    Version getVersion();
    VersionInfo getAllVersion();

    bool isBootedNonExclusive();

   private:
    // private
    DeviceInfo deviceInfo;

    void threadedStateMonitoring();

    std::thread stateMonitoringThread;

    tl::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl);

    tl::optional<std::string> saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename);

    // state of the session
    SessionState sessionState = SessionState::NOT_CREATED;

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    std::string sessionId;
};

}  // namespace dai

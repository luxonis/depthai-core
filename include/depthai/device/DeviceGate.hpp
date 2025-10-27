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
    enum class SessionState { NOT_CREATED, CREATED, RUNNING, STOPPED, STOPPING, CRASHED, DESTROYED, ERROR_STATE };

    struct CrashDump {
        std::vector<uint8_t> data;
        std::string filename;
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
    SessionState getState();
    // Waits for the gate session to end and tries to get the logs and crash dump out
    std::optional<CrashDump> waitForSessionEnd();

    std::optional<CrashDump> getCrashDump();

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

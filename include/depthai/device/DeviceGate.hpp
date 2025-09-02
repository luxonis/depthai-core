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
#include "depthai/xlink/XLinkStream.hpp"
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
    class GateImpl {
       public:
        virtual ~GateImpl();
        virtual bool isOkay() = 0;
        virtual bool createSession(std::string version, bool exclusive, XLinkPlatform_t platform, std::string& sessionId, std::atomic_bool& sessionCreated) = 0;
        virtual bool startSession(std::string sessionId) = 0;
        virtual bool stopSession(std::string sessionId) = 0;
        virtual bool deleteSession(std::string sessionId) = 0;
        virtual bool destroySession(std::string sessionId) = 0;
        virtual SessionState getState(std::string sessionId) = 0;
        virtual std::optional<CrashDump> waitForSessionEnd() = 0;
        virtual std::optional<CrashDump> getCrashDump() = 0;
        virtual Version getVersion() = 0;
        virtual VersionInfo getAllVersion() = 0;
        virtual bool isBootedNonExclusive() = 0;
        virtual std::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl, std::string& filename) = 0;
    };

    class USBImpl : public GateImpl {
       public:
        ~USBImpl();
        bool isOkay() override;
        bool createSession(std::string version, bool exclusive, XLinkPlatform_t platform, std::string& sessionId, std::atomic_bool& sessionCreated) override;
        bool startSession(std::string sessionId) override;
        bool stopSession(std::string sessionId) override;
        bool deleteSession(std::string sessionId) override;
        bool destroySession(std::string sessionId) override;
        SessionState getState(std::string sessionId) override;
        std::optional<CrashDump> waitForSessionEnd() override;
        std::optional<CrashDump> getCrashDump() override;
        Version getVersion() override;
        VersionInfo getAllVersion() override;
        bool isBootedNonExclusive() override;
        std::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl, std::string& filename) override;
    };

    class HTTPImpl : public GateImpl {
       public:
        HTTPImpl(DeviceInfo deviceInfo);
        ~HTTPImpl();
        bool isOkay() override;
        bool createSession(std::string version, bool exclusive, XLinkPlatform_t platform, std::string& sessionId, std::atomic_bool& sessionCreated) override;
        bool startSession(std::string sessionId) override;
        bool stopSession(std::string sessionId) override;
        bool deleteSession(std::string sessionId) override;
        bool destroySession(std::string sessionId) override;
        SessionState getState(std::string sessionId) override;
        std::optional<CrashDump> waitForSessionEnd() override;
        std::optional<CrashDump> getCrashDump() override;
        Version getVersion() override;
        VersionInfo getAllVersion() override;
        bool isBootedNonExclusive() override;
        std::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl, std::string& filename) override;

       private:
        // pimpl
        class Impl;
        Pimpl<Impl> pimpl;
    };

    std::shared_ptr<GateImpl> impl;

    DeviceInfo deviceInfo;

    std::thread stateMonitoringThread;

    std::optional<std::vector<uint8_t>> getFile(const std::string& fileUrl, std::string& filename);

    // state of the session
    std::atomic_bool sessionCreated{false};

    XLinkPlatform_t platform;
    std::shared_ptr<XLinkConnection> gateConnection;
    std::shared_ptr<XLinkStream> gateStream;
    std::string version;

    std::string sessionId;
};

}  // namespace dai

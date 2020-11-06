#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "pipeline/Pipeline.hpp"
#include "xlink/XLinkConnection.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace dai {

// DeviceBootloader (RAII), connects to device and maintains watchdog ...
class DeviceBootloader {
   public:
    // Bootloader version
    struct Version {
        explicit Version(const std::string& v);
        Version(unsigned major, unsigned minor, unsigned patch);
        bool operator==(const Version& other) const;
        bool operator>(const Version& other) const;
        bool operator<(const Version& other) const;
        std::string toString() const;

       private:
        unsigned major, minor, patch;
    };

    //// static API
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();
    static std::vector<DeviceInfo> getAllAvailableDevices();
    static std::vector<uint8_t> createDepthaiApplicationPackage(Pipeline& pipeline, std::string pathToCmd = "");
    static Version getEmbeddedBootloaderVersion();
    static std::vector<std::uint8_t> getEmbeddedBootloaderBinary();
    ////

    DeviceBootloader() = delete;
    explicit DeviceBootloader(const DeviceInfo& devInfo);
    DeviceBootloader(const DeviceInfo& devInfo, const char* pathToBootloader);
    DeviceBootloader(const DeviceInfo& devInfo, const std::string& pathToBootloader);
    ~DeviceBootloader();

    std::tuple<bool, std::string> flash(std::function<void(float)> progressCb, Pipeline& pipeline);
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package);
    std::tuple<bool, std::string> flashBootloader(std::function<void(float)> progressCb, std::string path = "");
    Version getVersion();
    bool isEmbeddedVersion();

   private:
    // private static

    // private variables
    void init(bool embeddedMvcmd, const std::string& pathToMvcmd);
    std::shared_ptr<XLinkConnection> connection;
    DeviceInfo deviceInfo = {};

    bool isEmbedded = false;

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};
};

}  // namespace dai

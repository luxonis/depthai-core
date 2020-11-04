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
#include "bootloader/Version.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace dai {

// DeviceFlasher (RAII), connects to device and maintains watchdog ...
class DeviceFlasher {
   public:
    //// static API
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();
    static std::vector<DeviceInfo> getAllAvailableDevices();
    static std::vector<uint8_t> createDepthaiFirmware(Pipeline& pipeline, std::string pathToCmd = "");
    static bootloader::Version getEmbeddedBootloaderVersion();
    ////

    DeviceFlasher() = deleted;
    explicit DeviceFlasher(const DeviceInfo& devInfo);
    DeviceFlasher(const DeviceInfo& devInfo, const char* pathToBootloader);
    DeviceFlasher(const DeviceInfo& devInfo, const std::string& pathToBootloader);
    ~DeviceFlasher();

    std::tuple<bool, std::string> flash(std::function<void(float)> progressCb, Pipeline& pipeline);    
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package);
    std::tuple<bool, std::string> flashBootloader(std::function<void(float)> progressCb, std::string path = "");

   private:
    // private static
    static std::vector<std::uint8_t> getEmbeddedBootloaderBinary();

    // private variables
    void init(bool embeddedMvcmd, const std::string& pathToMvcmd);
    std::shared_ptr<XLinkConnection> connection; 
    DeviceInfo deviceInfo = {};

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

};

}  // namespace dai

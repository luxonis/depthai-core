#pragma once

#include <cstdint>
#include <depthai/device/Device.hpp>
#include <depthai/device/DeviceBootloader.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "archive.h"

namespace dai {

namespace fs = std::filesystem;

class TarXzAccessor {
   public:
    TarXzAccessor(const std::vector<std::uint8_t>& tarGzFile);

    std::optional<std::vector<std::uint8_t>> getFile(const std::string& path) const;

   private:
    std::map<std::string, std::vector<std::uint8_t>> resourceMap;  // Path to file data
};

class Resources {
    Resources();
    ~Resources();

    mutable std::mutex mtxDevice;
    mutable std::condition_variable cvDevice;
    std::thread lazyThreadDevice;
    bool readyDevice;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapDevice;

    mutable std::mutex mtxBootloader;
    mutable std::condition_variable cvBootloader;
    std::thread lazyThreadBootloader;
    bool readyBootloader;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapBootloader;
    std::vector<std::uint8_t> getDeviceFwp(const std::string& fwPath, const std::string& envPath) const;

   public:
    static Resources& getInstance();
    Resources(Resources const&) = delete;
    void operator=(Resources const&) = delete;

    std::vector<std::uint8_t> getDeviceFirmware(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_UNIVERSAL) const;
    std::vector<std::uint8_t> getDeviceFirmware(Device::Config config, std::filesystem::path pathToMvcmd = std::filesystem::path()) const;
    std::vector<std::uint8_t> getBootloaderFirmware(DeviceBootloader::Type type = DeviceBootloader::Type::USB) const;
    std::vector<std::uint8_t> getDeviceRVC3Fwp() const;
    std::vector<std::uint8_t> getDeviceRVC4Fwp() const;
    TarXzAccessor getEmbeddedVisualizer() const;
};

}  // namespace dai

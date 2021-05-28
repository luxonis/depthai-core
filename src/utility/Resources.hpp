#pragma once

#include <mutex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <thread>

// project
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/device/DeviceBootloader.hpp>

namespace dai
{
class Resources {
    // private constructor
    Resources();
    ~Resources();

    std::mutex mtx;
    std::thread lazyThread;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMap;

    std::vector<std::uint8_t> getDeviceBinary(Device::Config config);

public:
    static Resources& getInstance();
    Resources(Resources const&) = delete;
    void operator=(Resources const&) = delete;

    // Available resources
    std::vector<std::uint8_t> getDeviceFirmware(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_2020_1);
    std::vector<std::uint8_t> getDeviceFirmware(Device::Config config);
    std::vector<std::uint8_t> getBootloaderFirmware();

};

} // namespace dai

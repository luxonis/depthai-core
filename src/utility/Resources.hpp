#pragma once

#include <mutex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <thread>

// project
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/device/DeviceBootloader.hpp>

namespace dai
{

class Resources {
    // private constructor
    Resources();
    ~Resources();

    std::mutex mtxDevice;
    std::thread lazyThreadDevice;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapDevice;

    std::mutex mtxBootloader;
    std::thread lazyThreadBootloader;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapBootloader;

    std::vector<std::uint8_t> getDeviceBinary(OpenVINO::Version version, bool usb2Mode);

public:
    static Resources& getInstance();
    Resources(Resources const&) = delete;
    void operator=(Resources const&) = delete;

    // Available resources
    std::vector<std::uint8_t> getDeviceFirmware(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_2020_1);
    std::vector<std::uint8_t> getBootloaderFirmware(DeviceBootloader::Type type = DeviceBootloader::Type::USB);

};

} // namespace dai

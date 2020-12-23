#pragma once

#include <mutex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <thread>

#include <depthai/openvino/OpenVINO.hpp>

namespace dai
{
    
class Resources {
    // private constructor
    Resources();
    ~Resources();

    std::mutex mtx;
    std::thread lazyThread;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMap;

    std::vector<std::uint8_t> getDeviceBinary(OpenVINO::Version version, bool usb2Mode);

public:
    static Resources& getInstance();
    Resources(Resources const&) = delete;
    void operator=(Resources const&) = delete;

    // Available resources
    std::vector<std::uint8_t> getDeviceFirmware(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_2020_1);
    std::vector<std::uint8_t> getBootloaderFirmware();

};

} // namespace dai

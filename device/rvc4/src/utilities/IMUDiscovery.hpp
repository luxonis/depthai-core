#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

// shared
// #include "depthai/common/IMUFeatures.hpp"

namespace dai {
namespace gate {

struct IMUDescription {
    static constexpr const int INVALID_GPIO = -1;
    std::string imuType = "NONE";
    std::string spiDevAddress = "";
    int interruptGpio = INVALID_GPIO;
    int resetGpio = INVALID_GPIO;
    int wakeGpio = INVALID_GPIO;
    int bootGpio = INVALID_GPIO;
    int csGpio = INVALID_GPIO;
};

class IMUDiscovery {
    // private constructor
    IMUDiscovery();
    ~IMUDiscovery() = default;

    mutable std::mutex mtx;
    IMUDescription imuDescription;

    std::vector<std::string> possibleIMUs = {"bno08x", "bmi270", "imu-discoverable", "accel", "gyro", "mag", "rotv", "game_rv"};

   public:
    static IMUDiscovery& getInstance();
    IMUDiscovery(IMUDiscovery const&) = delete;
    void operator=(IMUDiscovery const&) = delete;

    IMUDescription forceEnumerateIMU();
    IMUDescription getIMUDescription() const;
    void overrideConnectedIMU(const std::string& imu);
};

}  // namespace gate

}  // namespace dai

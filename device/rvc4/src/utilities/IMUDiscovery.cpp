#include "IMUDiscovery.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#define DEBUG 0

#ifndef DEBUG
    #define DEBUG 0
#endif

namespace dai {
namespace gate {

IMUDiscovery::IMUDiscovery() {
    forceEnumerateIMU();
}

IMUDiscovery& IMUDiscovery::getInstance() {
    static IMUDiscovery instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

IMUDescription IMUDiscovery::forceEnumerateIMU() {
    std::unique_lock<std::mutex> lock(mtx);

    bool found = false;  // atm we support only 1 IMU per board
    IMUDescription description;

    if(!found) {
        for(const auto& imu : possibleIMUs) {
            std::string sscCmd = "ssc_sensor_info -sensor=" + imu + " | grep " + imu;

            if(!system(sscCmd.c_str())) {
                description.imuType = imu;
                break;
            }
        }
    }

    imuDescription = description;

    return description;
}

IMUDescription IMUDiscovery::getIMUDescription() const {
    std::unique_lock<std::mutex> lock(mtx);
    return imuDescription;
}

void IMUDiscovery::overrideConnectedIMU(const std::string& imu) {
    std::unique_lock<std::mutex> lock(mtx);
    imuDescription.imuType = imu;
}

}  // namespace gate
}  // namespace dai

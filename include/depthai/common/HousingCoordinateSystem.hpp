// filepath: /home/tomas/code/depthai-device-kb/external/depthai-core/include/depthai/common/Housings.hpp
#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>

namespace dai {
/**
 * Which Housing to use.
 *
 * AUTO denotes that the decision will be made by device
 */
enum class HousingCoordinateSystem : int32_t {
    AUTO = -1,
    CAM_A,
    CAM_B,
    CAM_C,
    CAM_D,
    CAM_E,
    CAM_F,
    CAM_G,
    CAM_H,
    CAM_I,
    CAM_J,
    FRONT_CAM_A,
    FRONT_CAM_B,
    FRONT_CAM_C,
    FRONT_CAM_D,
    FRONT_CAM_E,
    FRONT_CAM_F,
    FRONT_CAM_G,
    FRONT_CAM_H,
    FRONT_CAM_I,
    FRONT_CAM_J,
    VESA_A,
    VESA_B,
    VESA_C,
    VESA_D,
    VESA_E,
    VESA_F,
    VESA_G,
    VESA_H,
    VESA_I,
    VESA_J,
    IMU,
};

inline std::string toString(HousingCoordinateSystem housing) {
    switch(housing) {
        case HousingCoordinateSystem::AUTO:
            return "AUTO";
        case HousingCoordinateSystem::CAM_A:
            return "CAM_A";
        case HousingCoordinateSystem::CAM_B:
            return "CAM_B";
        case HousingCoordinateSystem::CAM_C:
            return "CAM_C";
        case HousingCoordinateSystem::CAM_D:
            return "CAM_D";
        case HousingCoordinateSystem::CAM_E:
            return "CAM_E";
        case HousingCoordinateSystem::CAM_F:
            return "CAM_F";
        case HousingCoordinateSystem::CAM_G:
            return "CAM_G";
        case HousingCoordinateSystem::CAM_H:
            return "CAM_H";
        case HousingCoordinateSystem::CAM_I:
            return "CAM_I";
        case HousingCoordinateSystem::CAM_J:
            return "CAM_J";
        case HousingCoordinateSystem::FRONT_CAM_A:
            return "FRONT_CAM_A";
        case HousingCoordinateSystem::FRONT_CAM_B:
            return "FRONT_CAM_B";
        case HousingCoordinateSystem::FRONT_CAM_C:
            return "FRONT_CAM_C";
        case HousingCoordinateSystem::FRONT_CAM_D:
            return "FRONT_CAM_D";
        case HousingCoordinateSystem::FRONT_CAM_E:
            return "FRONT_CAM_E";
        case HousingCoordinateSystem::FRONT_CAM_F:
            return "FRONT_CAM_F";
        case HousingCoordinateSystem::FRONT_CAM_G:
            return "FRONT_CAM_G";
        case HousingCoordinateSystem::FRONT_CAM_H:
            return "FRONT_CAM_H";
        case HousingCoordinateSystem::FRONT_CAM_I:
            return "FRONT_CAM_I";
        case HousingCoordinateSystem::FRONT_CAM_J:
            return "FRONT_CAM_J";
        case HousingCoordinateSystem::VESA_A:
            return "VESA_A";
        case HousingCoordinateSystem::VESA_B:
            return "VESA_B";
        case HousingCoordinateSystem::VESA_C:
            return "VESA_C";
        case HousingCoordinateSystem::VESA_D:
            return "VESA_D";
        case HousingCoordinateSystem::VESA_E:
            return "VESA_E";
        case HousingCoordinateSystem::VESA_F:
            return "VESA_F";
        case HousingCoordinateSystem::VESA_G:
            return "VESA_G";
        case HousingCoordinateSystem::VESA_H:
            return "VESA_H";
        case HousingCoordinateSystem::VESA_I:
            return "VESA_I";
        case HousingCoordinateSystem::VESA_J:
            return "VESA_J";
        case HousingCoordinateSystem::IMU:
            return "IMU";
        default:
            return "UNKNOWN";
    }
}

#ifdef DEPTHAI_HAVE_HOUSING_COORDINATES

const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>>& getHousingCoordinates();

#else

static inline const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>>& getHousingCoordinates() {
    static std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> data = {};
    return data;
}

#endif

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::HousingCoordinateSystem& housing) {
    return out << dai::toString(housing);
}
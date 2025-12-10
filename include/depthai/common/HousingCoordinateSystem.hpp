// filepath: /home/tomas/code/depthai-device-kb/external/depthai-core/include/depthai/common/Housings.hpp
#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which Housing to use.
 *
 * AUTO denotes that the decision will be made by device
 */
enum class HousingCoordinateSystem : int32_t {
    AUTO = -1,
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
    FRONT_CAM_K,
    FRONT_CAM_L,
    FRONT_CAM_M,
    FRONT_CAM_N,
    FRONT_CAM_O,
    FRONT_CAM_P,
    FRONT_CAM_Q,
    FRONT_CAM_R,
    FRONT_CAM_S,
    FRONT_CAM_T,
    FRONT_CAM_U,
    FRONT_CAM_V,
    FRONT_CAM_W,
    FRONT_CAM_X,
    FRONT_CAM_Y,
    FRONT_CAM_Z,
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
    VESA_K,
    VESA_L,
    VESA_M,
    VESA_N,
    VESA_O,
    VESA_P,
    VESA_Q,
    VESA_R,
    VESA_S,
    VESA_T,
    VESA_U,
    VESA_V,
    VESA_W,
    VESA_X,
    VESA_Y,
    VESA_Z,
};

inline std::string toString(HousingCoordinateSystem housing) {
    switch(housing) {
        case HousingCoordinateSystem::AUTO:
            return "AUTO";
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
        case HousingCoordinateSystem::FRONT_CAM_K:
            return "FRONT_CAM_K";
        case HousingCoordinateSystem::FRONT_CAM_L:
            return "FRONT_CAM_L";
        case HousingCoordinateSystem::FRONT_CAM_M:
            return "FRONT_CAM_M";
        case HousingCoordinateSystem::FRONT_CAM_N:
            return "FRONT_CAM_N";
        case HousingCoordinateSystem::FRONT_CAM_O:
            return "FRONT_CAM_O";
        case HousingCoordinateSystem::FRONT_CAM_P:
            return "FRONT_CAM_P";
        case HousingCoordinateSystem::FRONT_CAM_Q:
            return "FRONT_CAM_Q";
        case HousingCoordinateSystem::FRONT_CAM_R:
            return "FRONT_CAM_R";
        case HousingCoordinateSystem::FRONT_CAM_S:
            return "FRONT_CAM_S";
        case HousingCoordinateSystem::FRONT_CAM_T:
            return "FRONT_CAM_T";
        case HousingCoordinateSystem::FRONT_CAM_U:
            return "FRONT_CAM_U";
        case HousingCoordinateSystem::FRONT_CAM_V:
            return "FRONT_CAM_V";
        case HousingCoordinateSystem::FRONT_CAM_W:
            return "FRONT_CAM_W";
        case HousingCoordinateSystem::FRONT_CAM_X:
            return "FRONT_CAM_X";
        case HousingCoordinateSystem::FRONT_CAM_Y:
            return "FRONT_CAM_Y";
        case HousingCoordinateSystem::FRONT_CAM_Z:
            return "FRONT_CAM_Z";
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
        case HousingCoordinateSystem::VESA_K:
            return "VESA_K";
        case HousingCoordinateSystem::VESA_L:
            return "VESA_L";
        case HousingCoordinateSystem::VESA_M:
            return "VESA_M";
        case HousingCoordinateSystem::VESA_N:
            return "VESA_N";
        case HousingCoordinateSystem::VESA_O:
            return "VESA_O";
        case HousingCoordinateSystem::VESA_P:
            return "VESA_P";
        case HousingCoordinateSystem::VESA_Q:
            return "VESA_Q";
        case HousingCoordinateSystem::VESA_R:
            return "VESA_R";
        case HousingCoordinateSystem::VESA_S:
            return "VESA_S";
        case HousingCoordinateSystem::VESA_T:
            return "VESA_T";
        case HousingCoordinateSystem::VESA_U:
            return "VESA_U";
        case HousingCoordinateSystem::VESA_V:
            return "VESA_V";
        case HousingCoordinateSystem::VESA_W:
            return "VESA_W";
        case HousingCoordinateSystem::VESA_X:
            return "VESA_X";
        case HousingCoordinateSystem::VESA_Y:
            return "VESA_Y";
        case HousingCoordinateSystem::VESA_Z:
            return "VESA_Z";
        default:
            return "UNKNOWN";
    }
}

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::HousingCoordinateSystem& housing) {
    return out << dai::toString(housing);
}
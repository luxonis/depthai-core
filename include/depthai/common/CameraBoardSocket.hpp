#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which Camera socket to use.
 *
 * AUTO denotes that the decision will be made by device
 */
enum class CameraBoardSocket : int32_t {
    AUTO = -1,
    CAM_A,
    CAM_B,
    CAM_C,
    CAM_D,
    VERTICAL = CAM_D,
    CAM_E,
    CAM_F,
    CAM_G,
    CAM_H,
    CAM_I,
    CAM_J,
    // Deprecated naming
    RGB [[deprecated]] = CAM_A,
    CENTER [[deprecated]] = CAM_A,
    LEFT [[deprecated]] = CAM_B,
    RIGHT [[deprecated]] = CAM_C,
};

inline std::string toString(CameraBoardSocket socket) {
    switch(socket) {
        case CameraBoardSocket::AUTO:
            return "AUTO";
        case CameraBoardSocket::CAM_A:
            return "CAM_A";
        case CameraBoardSocket::CAM_B:
            return "CAM_B";
        case CameraBoardSocket::CAM_C:
            return "CAM_C";
        case CameraBoardSocket::CAM_D:
            return "CAM_D";
        case CameraBoardSocket::CAM_E:
            return "CAM_E";
        case CameraBoardSocket::CAM_F:
            return "CAM_F";
        case CameraBoardSocket::CAM_G:
            return "CAM_G";
        case CameraBoardSocket::CAM_H:
            return "CAM_H";
        case CameraBoardSocket::CAM_I:
            return "CAM_I";
        case CameraBoardSocket::CAM_J:
            return "CAM_J";
        default:
            return "UNKNOWN";
    }
}

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraBoardSocket& socket) {
    return out << dai::toString(socket);
}

#pragma once
#include <cstdint>
#include <iostream>
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

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraBoardSocket& socket) {
    switch(socket) {
        case dai::CameraBoardSocket::AUTO:
            out << "AUTO";
            break;
        case dai::CameraBoardSocket::CAM_A:
            out << "CAM_A";
            break;
        case dai::CameraBoardSocket::CAM_B:
            out << "CAM_B";
            break;
        case dai::CameraBoardSocket::CAM_C:
            out << "CAM_C";
            break;
        case dai::CameraBoardSocket::CAM_D:
            out << "CAM_D";
            break;
        case dai::CameraBoardSocket::CAM_E:
            out << "CAM_E";
            break;
        case dai::CameraBoardSocket::CAM_F:
            out << "CAM_F";
            break;
        case dai::CameraBoardSocket::CAM_G:
            out << "CAM_G";
            break;
        case dai::CameraBoardSocket::CAM_H:
            out << "CAM_H";
            break;
        case dai::CameraBoardSocket::CAM_I:
            out << "CAM_I";
            break;
        case dai::CameraBoardSocket::CAM_J:
            out << "CAM_J";
            break;
    }
    return out;
}

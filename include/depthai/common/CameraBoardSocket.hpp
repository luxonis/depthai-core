#pragma once

#include <ostream>

#include "depthai-shared/common/CameraBoardSocket.hpp"

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraBoardSocket& socket) {
    switch(socket) {
        case dai::CameraBoardSocket::AUTO:
            out << "AUTO";
            break;
        case dai::CameraBoardSocket::RGB:
            out << "RGB/CENTER/CAM_A";
            break;
        case dai::CameraBoardSocket::LEFT:
            out << "LEFT/CAM_B";
            break;
        case dai::CameraBoardSocket::RIGHT:
            out << "RIGHT/CAM_C";
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
    }
    return out;
}

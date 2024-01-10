#pragma once

#include <ostream>

#include "depthai-shared/common/CameraBoardSocket.hpp"

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

namespace dai {

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

inline CameraBoardSocket fromString(const std::string& socket) {
    if(socket == "CAM_A") {
        return CameraBoardSocket::CAM_A;
    } else if(socket == "CAM_B") {
        return CameraBoardSocket::CAM_B;
    } else if(socket == "CAM_C") {
        return CameraBoardSocket::CAM_C;
    } else if(socket == "CAM_D") {
        return CameraBoardSocket::CAM_D;
    } else if(socket == "CAM_E") {
        return CameraBoardSocket::CAM_E;
    } else if(socket == "CAM_F") {
        return CameraBoardSocket::CAM_F;
    } else if(socket == "CAM_G") {
        return CameraBoardSocket::CAM_G;
    } else if(socket == "CAM_H") {
        return CameraBoardSocket::CAM_H;
    } else if(socket == "CAM_I") {
        return CameraBoardSocket::CAM_I;
    } else if(socket == "CAM_J") {
        return CameraBoardSocket::CAM_J;
    } else {
        return CameraBoardSocket::AUTO;
    }
}

}  // namespace dai

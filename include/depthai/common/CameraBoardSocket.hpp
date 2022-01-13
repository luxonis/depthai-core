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
            out << "RGB";
            break;
        case dai::CameraBoardSocket::LEFT:
            out << "LEFT";
            break;
        case dai::CameraBoardSocket::RIGHT:
            out << "RIGHT";
            break;
    }
    return out;
}

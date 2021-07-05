#pragma once

#include <ostream>

#include "depthai-shared/common/CameraBoardSocket.hpp"

namespace dai {

inline std::ostream& operator<<(std::ostream& out, const CameraBoardSocket& socket) {
    switch(socket) {
        case CameraBoardSocket::AUTO:
            out << "AUTO";
            break;
        case CameraBoardSocket::RGB:
            out << "RGB";
            break;
        case CameraBoardSocket::LEFT:
            out << "LEFT";
            break;
        case CameraBoardSocket::RIGHT:
            out << "RIGHT";
            break;
    }
    return out;
}

}  // namespace dai
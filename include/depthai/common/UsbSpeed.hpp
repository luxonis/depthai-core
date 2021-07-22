#pragma once

#include <ostream>

#include "depthai-shared/common/UsbSpeed.hpp"

namespace dai {

inline std::ostream& operator<<(std::ostream& out, const UsbSpeed& speed) {
    switch(speed) {
        case UsbSpeed::UNKNOWN:
            out << "UNKNOWN";
            break;
        case UsbSpeed::LOW:
            out << "LOW";
            break;
        case UsbSpeed::FULL:
            out << "FULL";
            break;
        case UsbSpeed::HIGH:
            out << "HIGH";
            break;
        case UsbSpeed::SUPER:
            out << "SUPER";
            break;
        case UsbSpeed::SUPER_PLUS:
            out << "SUPER_PLUS";
            break;
    }
    return out;
}

}  // namespace dai

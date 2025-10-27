#pragma once

// std
#include <cstdint>
#include <ostream>

namespace dai {

/**
 * Get USB Speed
 */

enum class UsbSpeed : int32_t { UNKNOWN, LOW, FULL, HIGH, SUPER, SUPER_PLUS };

}  // namespace dai

/**
 * Get USB Speed
 */
inline std::ostream& operator<<(std::ostream& out, const dai::UsbSpeed& usbSpeed) {
    switch(usbSpeed) {
        case dai::UsbSpeed::UNKNOWN:
            out << "UNKNOWN";
            break;
        case dai::UsbSpeed::LOW:
            out << "LOW";
            break;
        case dai::UsbSpeed::FULL:
            out << "FULL";
            break;
        case dai::UsbSpeed::HIGH:
            out << "HIGH";
            break;
        case dai::UsbSpeed::SUPER:
            out << "SUPER";
            break;
        case dai::UsbSpeed::SUPER_PLUS:
            out << "SUPER_PLUS";
            break;
    }
    return out;
}
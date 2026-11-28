#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which external frame sync role the device should have.
 *
 * AUTO_DETECT denotes that the decision will be made by device.
 * It will choose between MASTER and SLAVE.
 */
enum class ExternalFrameSyncRole : int32_t {
    AUTO_DETECT = -1,

    MASTER,

    SLAVE,
};

inline std::string toString(ExternalFrameSyncRole role) {
    switch(role) {
        case ExternalFrameSyncRole::AUTO_DETECT:
            return "AUTO DETECT";
        case ExternalFrameSyncRole::MASTER:
            return "MASTER";
        case ExternalFrameSyncRole::SLAVE:
            return "SLAVE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace dai

inline std::ostream& operator<<(std::ostream& out, const dai::ExternalFrameSyncRole& socket) {
    return out << dai::toString(socket);
}

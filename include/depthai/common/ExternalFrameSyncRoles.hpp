#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which Fsync role the device will have.
 *
 * AUTO_DETECT denotes that the decision will be made by device.
 * It will choose between MASTER and SLAVE.
 */
enum class ExternalFrameSyncRole : int32_t {
    // This role is used in setExternalFrameSyncRole() function.
    // It signals to the device to automatically detect External frame sync configuration.
    AUTO_DETECT = -1,

    // Fsync master.
    // Device generates frame sync signal and outputs it to other devices.
    MASTER,

    // Fsync slave.
    // Device must lock onto an external frame sync signal.
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

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::ExternalFrameSyncRole& socket) {
    return out << dai::toString(socket);
}

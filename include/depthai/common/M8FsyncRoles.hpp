#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which Fsymc role the device will have.
 *
 * AUTO_DETECT denotes that the decision will be made by device.
 * It will choose between MASTER and SLAVE.
 */
enum class M8FsyncRole : int32_t {
    // This role is only used in setM8FsyncRole() function.
    // It signals to the device to automatically detect M8 Fsync configuration.
    AUTO_DETECT = -1,

    // Fsync master.
    // STM generates Fsync signal and outputs it through M8 connector.
    MASTER,

    // Fsync slave.
    // Device must lock onto an external Fsync signal on the M8 connector.
    SLAVE,
};

inline std::string toString(M8FsyncRole role) {
    switch(role) {
        case M8FsyncRole::AUTO_DETECT:
            return "AUTO DETECT";
        case M8FsyncRole::MASTER:
            return "MASTER";
        case M8FsyncRole::SLAVE:
            return "SLAVE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::M8FsyncRole& socket) {
    return out << dai::toString(socket);
}

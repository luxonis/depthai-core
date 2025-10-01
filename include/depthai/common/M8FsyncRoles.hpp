#pragma once
#include <cstdint>
#include <iostream>
#include <string>

namespace dai {
/**
 * Which Fsymc role the device will have.
 *
 * AUTO_DETECT denotes that the decision will be made by device.
 * It will choose between MASTER_WITH_OUTPUT and SLAVE.
 */
enum class M8FsyncRole : int32_t {
    AUTO_DETECT = -1,
    MASTER_WITHOUT_OUTPUT,
    MASTER_WITH_OUTPUT,
    SLAVE,
};

inline std::string toString(M8FsyncRole role) {
    switch(role) {
        case M8FsyncRole::AUTO_DETECT:
            return "AUTO DETECT";
        case M8FsyncRole::MASTER_WITHOUT_OUTPUT:
            return "MASTER, NO OUTPUT";
        case M8FsyncRole::MASTER_WITH_OUTPUT:
            return "MASTER, WITH OUTPUT";
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

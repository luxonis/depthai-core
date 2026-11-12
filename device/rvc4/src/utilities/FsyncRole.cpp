#include "utilities/FsyncRole.hpp"

#include <stdexcept>
#include <string>

namespace dai {
namespace gate {

ExternalFrameSyncRole resolveRole(ExternalFrameSyncRole requested, bool supportsExternalSync, std::optional<M8ConnectorState> state) {
    switch(requested) {
        case ExternalFrameSyncRole::AUTO_DETECT:
            return supportsExternalSync && state == M8ConnectorState::CONNECTED ? ExternalFrameSyncRole::SLAVE : ExternalFrameSyncRole::MASTER;
        case ExternalFrameSyncRole::MASTER:
            return ExternalFrameSyncRole::MASTER;
        case ExternalFrameSyncRole::SLAVE:
            return supportsExternalSync ? ExternalFrameSyncRole::SLAVE : ExternalFrameSyncRole::MASTER;
        default:
            throw std::invalid_argument("Invalid external frame sync role specified.");
    }
}

uint32_t configFor(ExternalFrameSyncRole resolved) {
    switch(resolved) {
        case ExternalFrameSyncRole::MASTER:
            return FSYNC_STM_CONFIG_MASTER_MODE_WITH_OUTPUT;
        case ExternalFrameSyncRole::SLAVE:
            return FSYNC_STM_CONFIG_SLAVE_MODE;
        default:
            throw std::invalid_argument("Cannot convert role " + dai::toString(resolved) + " to register value!");
    }
}

ExternalFrameSyncRole effectiveRole(const RoleState& state) {
    return state.resolved;
}

bool externalRoleForced(const RoleState& state) {
    return state.requested == ExternalFrameSyncRole::MASTER || state.requested == ExternalFrameSyncRole::SLAVE;
}

}  // namespace gate
}  // namespace dai

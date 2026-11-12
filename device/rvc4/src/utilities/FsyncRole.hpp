#pragma once

#include <cstdint>
#include <optional>
#include <variant>

#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "utilities/OSUtils.hpp"

namespace dai {
namespace gate {

uint32_t constexpr FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT = 0;
uint32_t constexpr FSYNC_STM_CONFIG_MASTER_MODE_WITH_OUTPUT = 1;
uint32_t constexpr FSYNC_STM_CONFIG_SLAVE_MODE = 2;

struct Applied {};
struct ControllerFailure {
    int err;
};
using SetInfoResult = std::variant<Applied, ControllerFailure>;

struct RoleState {
    ExternalFrameSyncRole requested;
    ExternalFrameSyncRole resolved;
    SetInfoResult result;
};

ExternalFrameSyncRole resolveRole(ExternalFrameSyncRole requested, bool supportsExternalSync, std::optional<M8ConnectorState> state);
uint32_t configFor(ExternalFrameSyncRole resolved);
ExternalFrameSyncRole effectiveRole(const RoleState& state);
bool externalRoleForced(const RoleState& state);

}  // namespace gate
}  // namespace dai

#pragma once

#include <sys/types.h>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "utilities/OSUtils.hpp"

namespace dai {
namespace gate {

uint32_t constexpr FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT = 0;
uint32_t constexpr FSYNC_STM_CONFIG_MASTER_MODE_WITH_OUTPUT = 1;
uint32_t constexpr FSYNC_STM_CONFIG_SLAVE_MODE = 2;

enum class PinProblem { Busy, ConnectorNotReady, ConnectorFailed };

struct ProcessHolder {
    pid_t pid;
    std::string comm;  // "" when /proc/<pid>/comm cannot be read
};
struct KernelHolder {
    std::string name;
};
struct LineHolder {
    std::string line;
    std::string consumer;  // "" when unknown
};
using Holder = std::variant<ProcessHolder, KernelHolder, LineHolder>;

struct Applied {};
// The aux connector did not give fsync-stm the FSYNC pin, so the STM stays in CONFIG 0
struct PinUnavailable {
    PinProblem problem;
    std::string pinLabel;          // "" when the connector cannot be read
    std::optional<Holder> holder;  // Busy only, empty when the connector shows no holder
};
struct ControllerFailure {
    int err;
};
using SetInfoResult = std::variant<Applied, PinUnavailable, ControllerFailure>;

struct RoleState {
    ExternalFrameSyncRole requested;
    ExternalFrameSyncRole resolved;
    SetInfoResult result;
};

ExternalFrameSyncRole resolveRole(ExternalFrameSyncRole requested, bool supportsExternalSync, std::optional<M8ConnectorState> state);
uint32_t configFor(ExternalFrameSyncRole resolved);
ExternalFrameSyncRole effectiveRole(const RoleState& state);
bool externalRoleForced(const RoleState& state);

/**
 * Classify a failed SET_INFO of CONFIG 1 or 2.
 * @param linked The fsync-stm node links to an enabled aux connector.
 * @param bound The aux connector driver is bound.
 * @return The pin problem, or std::nullopt for a controller failure.
 */
std::optional<PinProblem> classifySetInfoError(int err, bool linked, bool bound);

std::string pinReason(const PinUnavailable& unavailable);
std::optional<std::string> autoWarning(const RoleState& state);
std::optional<std::string> setRoleError(const RoleState& state);
std::optional<std::string> strobeNeedsPinError(const RoleState& state);

}  // namespace gate
}  // namespace dai

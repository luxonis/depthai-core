#include "utilities/FsyncRole.hpp"

#include <cerrno>
#include <cstring>
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
        case ExternalFrameSyncRole::STANDALONE:
            return ExternalFrameSyncRole::STANDALONE;
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
        case ExternalFrameSyncRole::STANDALONE:
            return FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT;
        default:
            throw std::invalid_argument("Cannot convert role " + dai::toString(resolved) + " to register value!");
    }
}

ExternalFrameSyncRole effectiveRole(const RoleState& state) {
    return std::holds_alternative<PinUnavailable>(state.result) ? ExternalFrameSyncRole::STANDALONE : state.resolved;
}

bool externalRoleForced(const RoleState& state) {
    return state.requested == ExternalFrameSyncRole::MASTER || state.requested == ExternalFrameSyncRole::SLAVE;
}

std::optional<PinProblem> classifySetInfoError(int err, bool linked, bool bound, std::optional<uint32_t> config) {
    if(!linked || config != FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT) {
        return std::nullopt;
    }
    switch(err) {
        case EBUSY:
            return PinProblem::Busy;
        case EAGAIN:
            return PinProblem::ConnectorNotReady;
        case EIO:
            // With the connector bound, EIO means the STM kept another CONFIG
            return bound ? std::nullopt : std::optional<PinProblem>(PinProblem::ConnectorFailed);
        default:
            return std::nullopt;
    }
}

namespace {

struct HolderText {
    std::string operator()(const ProcessHolder& holder) const {
        std::string text = "is held by process " + std::to_string(holder.pid);
        return holder.comm.empty() ? text : text + " (" + holder.comm + ")";
    }
    std::string operator()(const KernelHolder& holder) const {
        return "is held by kernel driver " + holder.name;
    }
    std::string operator()(const LineHolder& holder) const {
        std::string text = "is in use as GPIO line " + holder.line;
        return holder.consumer.empty() ? text : text + " by \"" + holder.consumer + "\"";
    }
};

}  // namespace

std::string holderText(const Holder& holder) {
    return std::visit(HolderText{}, holder);
}

std::string pinReason(const PinUnavailable& unavailable) {
    switch(unavailable.problem) {
        case PinProblem::Busy:
            if(unavailable.pinLabel.empty()) {
                return "the FSYNC pin is busy";
            }
            return "connector pin " + unavailable.pinLabel + " (FSYNC) " + (unavailable.holder ? holderText(*unavailable.holder) : "is busy");
        case PinProblem::ConnectorNotReady:
            return "the aux connector driver has not started yet";
        case PinProblem::ConnectorFailed:
            return "the aux connector driver failed to start, see dmesg";
    }
    throw std::invalid_argument("Unknown FSYNC pin problem");
}

std::optional<std::string> autoWarning(const RoleState& state) {
    const auto* unavailable = std::get_if<PinUnavailable>(&state.result);
    if(state.requested != ExternalFrameSyncRole::AUTO_DETECT || !unavailable) {
        return std::nullopt;
    }
    std::string warning = "External frame sync is off for this session: " + pinReason(*unavailable) + ". On-board camera sync stays on.";
    if(unavailable->problem == PinProblem::Busy) {
        warning += " Set the external frame sync role to STANDALONE to leave the FSYNC pin free without this warning.";
    }
    return warning;
}

std::optional<std::string> setRoleError(const RoleState& state) {
    if(state.requested == ExternalFrameSyncRole::AUTO_DETECT) {
        return std::nullopt;
    }
    std::string prefix = "Cannot set external frame sync role " + dai::toString(state.requested) + ": ";
    if(const auto* unavailable = std::get_if<PinUnavailable>(&state.result)) {
        std::string error = prefix + pinReason(*unavailable) + ".";
        if(unavailable->problem == PinProblem::Busy) {
            error += " Free the pin and set the role again, or use STANDALONE.";
        }
        return error;
    }
    if(const auto* failure = std::get_if<ControllerFailure>(&state.result)) {
        return prefix + "fsync-stm SET_INFO failed: " + std::strerror(failure->err) + " (" + std::to_string(failure->err) + ")";
    }
    if(const auto* failure = std::get_if<DeviceOpenFailure>(&state.result)) {
        return prefix + "the fsync-stm device could not be opened: " + std::strerror(failure->err) + " (" + std::to_string(failure->err) + ").";
    }
    if(std::holds_alternative<EarlierControllerFailure>(state.result)) {
        return prefix + "the frame sync controller failed earlier in this session.";
    }
    return std::nullopt;
}

std::optional<std::string> strobeNeedsPinError(const RoleState& state) {
    const auto* unavailable = std::get_if<PinUnavailable>(&state.result);
    if(!unavailable) {
        return std::nullopt;
    }
    return "External strobe needs the FSYNC pin: " + pinReason(*unavailable) + ".";
}

}  // namespace gate
}  // namespace dai

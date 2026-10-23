#pragma once

#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * State of the lock of the device's FSYNC period to the PTP time grid
 */
struct PtpFrameSyncStatus {
    enum class State : int32_t { IDLE, ACQUIRING, LOCKED, HOLDOVER, FAULT };

    State state = State::IDLE;
    /// Distance of the anchor camera's last FSYNC edge from its grid slot
    int64_t lastErrorNs = 0;
    /// Last FSYNC period commanded to the STM32, in timer cycles
    uint32_t appliedCycles = 0;
};

DEPTHAI_SERIALIZE_EXT(PtpFrameSyncStatus, state, lastErrorNs, appliedCycles);

}  // namespace dai

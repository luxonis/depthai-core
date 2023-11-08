#pragma once

#include <vector>

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync
 */
struct SyncProperties : PropertiesSerializable<Properties, SyncProperties> {
    /**
     * Optional manual sync threshold.
     * If not specified default threshold is obtained as:
     * thresholdMS = 1000.f / (minimum FPS of input frames) / 2
     * Frame timestamp difference below this threshold are considered synced.
     * 0 is not recommended in real time system, as frame interrupts are received
     * at slightly different time, even with perfect hardware sync.
     * 0 can be used when replaying frames.
     */
    tl::optional<float> syncThresholdMs;
};

DEPTHAI_SERIALIZE_EXT(SyncProperties, syncThresholdMs);

}  // namespace dai

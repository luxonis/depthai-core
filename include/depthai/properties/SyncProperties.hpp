#pragma once

#include <sys/types.h>

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct SyncProperties : PropertiesSerializable<Properties, SyncProperties> {
    /**
     * The maximal interval the messages can be apart in nanoseconds.
     */
    int64_t syncThresholdNs = 10e6;

    /**
     * The number of syncing attempts before fail (num of replaced messages).
     */
    int32_t syncAttempts = -1;
};

DEPTHAI_SERIALIZE_EXT(SyncProperties, syncThresholdNs, syncAttempts);

}  // namespace dai

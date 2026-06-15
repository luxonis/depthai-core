#pragma once

#include <sys/types.h>

#include "depthai/common/ProcessorType.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct SyncProperties : PropertiesSerializable<Properties, SyncProperties> {
    enum class TimestampSource : uint8_t {
        DEFAULT,
        DEVICE,
        HOST,
        SYSTEM
    };

    /**
     * The maximal interval the messages can be apart in nanoseconds.
     */
    int64_t syncThresholdNs = 10e6;

    /**
     * The number of syncing attempts before fail (num of replaced messages).
     */
    int32_t syncAttempts = -1;

    /**
     * Which processor should execute the node.
     */
    ProcessorType processor = ProcessorType::LEON_CSS;

    /**
     * Which timestamp to use for synchronization. On device the default is DEVICE, on host the default is HOST
     */
    TimestampSource timestampSource = TimestampSource::DEFAULT;

    ~SyncProperties() override;
};

DEPTHAI_SERIALIZE_EXT(SyncProperties, syncThresholdNs, syncAttempts, processor, timestampSource);

}  // namespace dai

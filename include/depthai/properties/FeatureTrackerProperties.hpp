#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"
#endif

/**
 * Specify properties for FeatureTracker
 */
struct FeatureTrackerProperties : PropertiesSerializable<Properties, FeatureTrackerProperties> {
    /**
     * Initial feature tracker config
     */
    FeatureTrackerConfig initialConfig;

    /**
     * Number of shaves reserved for feature tracking.
     * Optical flow can use 1 or 2 shaves, while for corner detection only 1 is enough.
     * Hardware motion estimation doesn't require shaves.
     * Maximum 2, minimum 1.
     */
    std::int32_t numShaves = 1;

    /**
     * Number of memory slices reserved for feature tracking.
     * Optical flow can use 1 or 2 memory slices, while for corner detection only 1 is enough.
     * Maximum number of features depends on the number of allocated memory slices.
     * Hardware motion estimation doesn't require memory slices.
     * Maximum 2, minimum 1.
     */
    std::int32_t numMemorySlices = 1;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

DEPTHAI_SERIALIZE_EXT(FeatureTrackerProperties, initialConfig, numShaves, numMemorySlices);

}  // namespace dai

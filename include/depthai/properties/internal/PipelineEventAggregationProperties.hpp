#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct PipelineEventAggregationProperties : PropertiesSerializable<Properties, PipelineEventAggregationProperties> {
    uint32_t aggregationWindowSize = 100;
    uint32_t statsUpdateIntervalMs = 1000;
    uint32_t eventWaitWindow = 16;
};

DEPTHAI_SERIALIZE_EXT(PipelineEventAggregationProperties, aggregationWindowSize, statsUpdateIntervalMs, eventWaitWindow);

}  // namespace dai

#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct PipelineEventAggregationProperties : PropertiesSerializable<Properties, PipelineEventAggregationProperties> {
    uint32_t aggregationWindowSize = 100;
    uint32_t eventBatchSize = 50; 
    uint32_t eventWaitWindow = 16;
};

DEPTHAI_SERIALIZE_EXT(PipelineEventAggregationProperties, aggregationWindowSize, eventBatchSize, eventWaitWindow);

}  // namespace dai

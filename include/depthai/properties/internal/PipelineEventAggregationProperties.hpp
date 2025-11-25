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
    bool traceOutput = false;

    ~PipelineEventAggregationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(PipelineEventAggregationProperties, aggregationWindowSize, statsUpdateIntervalMs, eventWaitWindow, traceOutput);

}  // namespace dai

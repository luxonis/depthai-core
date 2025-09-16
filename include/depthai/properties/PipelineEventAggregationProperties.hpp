#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct PipelineEventAggregationProperties : PropertiesSerializable<Properties, PipelineEventAggregationProperties> {
    int dummy = 0;
};

DEPTHAI_SERIALIZE_EXT(PipelineEventAggregationProperties, dummy);

}  // namespace dai

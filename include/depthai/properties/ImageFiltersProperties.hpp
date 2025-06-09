#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>
#include <vector>

#include "depthai/common/variant.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

using MedianFilterParams = dai::filters::params::MedianFilter;
using SpatialFilterParams = dai::filters::params::SpatialFilter;
using SpeckleFilterParams = dai::filters::params::SpeckleFilter;
using TemporalFilterParams = dai::filters::params::TemporalFilter;

// union of all filter params
typedef std::variant<MedianFilterParams, SpatialFilterParams, SpeckleFilterParams, TemporalFilterParams> FilterParams;

/**
 * Properties for StereoDepthFilterPipeline node
 */
struct ImageFiltersProperties : PropertiesSerializable<Properties, ImageFiltersProperties> {
    /**
     * List of filters (the type of which is determined by the filter parameters) to apply to the input frame
     */
    std::vector<FilterParams> filters;
};

DEPTHAI_SERIALIZE_EXT(ImageFiltersProperties, filters);

/**
 * Properties for DepthConfidenceFilter node
 */
struct ToFDepthConfidenceFilterProperties : PropertiesSerializable<Properties, ToFDepthConfidenceFilterProperties> {
    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold = 0.0f;
};

DEPTHAI_SERIALIZE_EXT(ToFDepthConfidenceFilterProperties, confidenceThreshold);

}  // namespace dai
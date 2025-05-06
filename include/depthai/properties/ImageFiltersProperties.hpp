#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>
#include <vector>

#include "depthai/common/variant.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {


struct MedianFilterParams {
    bool enable = false;
    dai::StereoDepthConfig::MedianFilter median = dai::StereoDepthConfig::MedianFilter::KERNEL_3x3;
};
DEPTHAI_SERIALIZE_EXT(MedianFilterParams, enable, median);

typedef dai::StereoDepthConfig::PostProcessing::SpatialFilter SpatialFilterParams;
typedef dai::StereoDepthConfig::PostProcessing::SpeckleFilter SpeckleFilterParams;
typedef dai::StereoDepthConfig::PostProcessing::TemporalFilter TemporalFilterParams;
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
struct DepthConfidenceFilterProperties : PropertiesSerializable<Properties, DepthConfidenceFilterProperties> {
    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold = 0.0f;
};

DEPTHAI_SERIALIZE_EXT(DepthConfidenceFilterProperties, confidenceThreshold);

}  // namespace dai
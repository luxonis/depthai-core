#pragma once

#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>

#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

using MedianFilterParams = dai::filters::params::MedianFilter;
using SpatialFilterParams = dai::filters::params::SpatialFilter;
using SpeckleFilterParams = dai::filters::params::SpeckleFilter;
using TemporalFilterParams = dai::filters::params::TemporalFilter;

// union of all filter params
typedef std::variant<MedianFilterParams, SpatialFilterParams, SpeckleFilterParams, TemporalFilterParams> FilterParams;

class ImageFiltersConfig : public Buffer {
   public:
    virtual ~ImageFiltersConfig() = default;

    /**
     * Insert filter parameters describing how a filter at index filterIndex should be updated
     * @param filterIndex Index of the filter to be inserted
     * @param filterParams Parameters of the filter to be inserted
     */
    ImageFiltersConfig& updateFilterAtIndex(std::int32_t filterIndex, FilterParams filterParams);

    /**
     * Insert filter parameters describing how a new filter should be inserted
     * @param filterParams Parameters of the filter to be inserted
     */
    ImageFiltersConfig& insertFilter(FilterParams filterParams);

    /**
     * Index of the filter to be applied
     */
    std::vector<std::int32_t> filterIndices = {};

    /**
     * Parameters of the filter to be applied
     */
    std::vector<FilterParams> filterParams = {};

    DEPTHAI_SERIALIZE(ImageFiltersConfig, filterIndices, filterParams);
};

class ToFDepthConfidenceFilterConfig : public Buffer {
   public:
    virtual ~ToFDepthConfidenceFilterConfig() = default;

    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold = 1.0f;

    DEPTHAI_SERIALIZE(ToFDepthConfidenceFilterConfig, confidenceThreshold);
};

}  // namespace dai
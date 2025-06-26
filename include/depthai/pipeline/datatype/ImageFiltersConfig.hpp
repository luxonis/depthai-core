#pragma once

#include <depthai/pipeline/datatype/StereoDepthConfig.hpp>

#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

using MedianFilterParams = dai::filters::params::MedianFilter;
using SpatialFilterParams = dai::filters::params::SpatialFilter;
using SpeckleFilterParams = dai::filters::params::SpeckleFilter;
using TemporalFilterParams = dai::filters::params::TemporalFilter;

// union of all filter params
typedef std::variant<MedianFilterParams, SpatialFilterParams, SpeckleFilterParams, TemporalFilterParams> FilterParams;


class ImageFiltersConfig : public Buffer {
   public:
    ImageFiltersConfig() = default;
    virtual ~ImageFiltersConfig() = default;

    /**
     * Insert filter parameters describing how a filter at index filterIndex should be updated
     * @param filterIndex Index of the filter to be inserted
     * @param filterParams Parameters of the filter to be inserted
     */
    ImageFiltersConfig& updateFilterAtIndex(std::int32_t filterIndex, FilterParams filterParams) {
        DAI_CHECK_V(this->filterIndices.size() == this->filterParams.size(),
                    "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
        this->filterIndices.push_back(filterIndex);
        this->filterParams.push_back(filterParams);
        return *this;
    }

    /**
     * Insert filter parameters describing how a new filter should be inserted
     * @param filterParams Parameters of the filter to be inserted
     */
    ImageFiltersConfig& insertFilter(FilterParams filterParams) {
        DAI_CHECK_V(filterIndices.size() == 0, "ImageFiltersConfig can either be used to create a new filter pipeline or update an existing one, not both");
        this->filterParams.push_back(filterParams);
        return *this;
    }

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
    ToFDepthConfidenceFilterConfig() = default;
    virtual ~ToFDepthConfidenceFilterConfig() = default;

    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold = 1.0f;

    DEPTHAI_SERIALIZE(ToFDepthConfidenceFilterConfig, confidenceThreshold);
};

}  // namespace dai
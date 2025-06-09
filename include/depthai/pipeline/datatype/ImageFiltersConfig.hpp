#pragma once

#include "depthai/properties/ImageFiltersProperties.hpp"
#include "depthai/common/variant.hpp"

namespace dai {

class ImageFiltersConfig : public Buffer {
   public:
    ImageFiltersConfig() = default;
    virtual ~ImageFiltersConfig() = default;

    /**
     * Index of the filter to be applied
     */
    std::int32_t filterIndex;

    /**
     * Parameters of the filter to be applied
     */
    FilterParams filterParams;

    DEPTHAI_SERIALIZE(ImageFiltersConfig, filterIndex, filterParams);
};

class ToFDepthConfidenceFilterConfig : public Buffer {
   public:
    ToFDepthConfidenceFilterConfig() = default;
    virtual ~ToFDepthConfidenceFilterConfig() = default;

    /**
     * Threshold for the confidence filter
     */
    float confidenceThreshold;

    DEPTHAI_SERIALIZE(ToFDepthConfidenceFilterConfig, confidenceThreshold);
};

}  // namespace dai
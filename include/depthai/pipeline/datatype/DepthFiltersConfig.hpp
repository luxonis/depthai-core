#pragma once

#include "depthai/properties/DepthFiltersProperties.hpp"
#include "depthai/common/variant.hpp"

namespace dai {

class SequentialDepthFiltersConfig : public Buffer {
   public:
    SequentialDepthFiltersConfig() = default;
    virtual ~SequentialDepthFiltersConfig() = default;

    /**
     * Index of the filter to be applied
     */
    std::int32_t filterIndex;

    /**
     * Parameters of the filter to be applied
     */
    FilterParams filterParams;

    DEPTHAI_SERIALIZE(SequentialDepthFiltersConfig, filterIndex, filterParams);
};

}  // namespace dai
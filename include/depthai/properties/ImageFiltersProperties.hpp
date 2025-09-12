#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct ImageFiltersProperties : PropertiesSerializable<Properties, ImageFiltersProperties> {
    /**
     * Initial config for the filter pipeline
     */
    ImageFiltersConfig initialConfig;

#if defined(__clang__)
    ~ImageFiltersProperties() override;
#else
    virtual ~ImageFiltersProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(ImageFiltersProperties, initialConfig);

struct ToFDepthConfidenceFilterProperties : PropertiesSerializable<Properties, ToFDepthConfidenceFilterProperties> {
    /**
     * Initial config for the ToF depth confidence filter
     */
    ToFDepthConfidenceFilterConfig initialConfig;

#if defined(__clang__)
    ~ToFDepthConfidenceFilterProperties() override;
#else
    virtual ~ToFDepthConfidenceFilterProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(ToFDepthConfidenceFilterProperties, initialConfig);

}  // namespace dai
#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

struct ImageFiltersProperties : PropertiesSerializable<Properties, ImageFiltersProperties> {
    /**
     * Initial config for the filter pipeline
     */
    ImageFiltersConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(ImageFiltersProperties, initialConfig);

#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

struct ToFDepthConfidenceFilterProperties : PropertiesSerializable<Properties, ToFDepthConfidenceFilterProperties> {
    /**
     * Initial config for the ToF depth confidence filter
     */
    ToFDepthConfidenceFilterConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(ToFDepthConfidenceFilterProperties, initialConfig);

#pragma clang diagnostic pop

}  // namespace dai
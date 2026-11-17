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

    ~ImageFiltersProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(ImageFiltersProperties, initialConfig);

struct ToFDepthConfidenceFilterProperties : PropertiesSerializable<Properties, ToFDepthConfidenceFilterProperties> {
    /**
     * Initial config for the ToF depth confidence filter
     */
    ToFDepthConfidenceFilterConfig initialConfig;

    ~ToFDepthConfidenceFilterProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(ToFDepthConfidenceFilterProperties, initialConfig);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::ImageFiltersProperties>;
extern template struct dai::PropertiesSerializable<dai::Properties, dai::ToFDepthConfidenceFilterProperties>;

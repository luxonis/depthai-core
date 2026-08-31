#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for CropConfigGenerator.
 */
struct CropConfigGeneratorProperties : PropertiesSerializable<Properties, CropConfigGeneratorProperties> {
    // Reserved for future device-side configuration.
    int dummy = 0;

    ~CropConfigGeneratorProperties() override;
};

DEPTHAI_SERIALIZE_EXT(CropConfigGeneratorProperties, dummy);

}  // namespace dai

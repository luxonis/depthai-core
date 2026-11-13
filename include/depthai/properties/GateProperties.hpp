#pragma once

#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Gate.
 */

struct GateProperties : PropertiesSerializable<Properties, GateProperties> {
    GateControl initialConfig;

    ~GateProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(GateProperties, initialConfig);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::GateProperties>;

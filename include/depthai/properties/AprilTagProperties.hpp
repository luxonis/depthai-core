#pragma once

#include <depthai/pipeline/datatype/AprilTagConfig.hpp>
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for AprilTag
 */
struct AprilTagProperties : PropertiesSerializable<Properties, AprilTagProperties> {
    AprilTagConfig initialConfig;

    /// Whether to wait for config at 'inputConfig' IO
    bool inputConfigSync = false;
};

DEPTHAI_SERIALIZE_EXT(AprilTagProperties, initialConfig, inputConfigSync);

}  // namespace dai

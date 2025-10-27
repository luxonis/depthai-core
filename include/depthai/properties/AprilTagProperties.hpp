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

    /// How many threads to use for AprilTag detection
    int numThreads = 1;

    ~AprilTagProperties() override;
};

DEPTHAI_SERIALIZE_EXT(AprilTagProperties, initialConfig, inputConfigSync);

}  // namespace dai

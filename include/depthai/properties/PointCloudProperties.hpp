#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wweak-vtables"

/**
 * Specify properties for PointCloud
 */
struct PointCloudProperties : PropertiesSerializable<Properties, PointCloudProperties> {
    PointCloudConfig initialConfig;

    int numFramesPool = 4;
};

#pragma clang diagnostic pop

DEPTHAI_SERIALIZE_EXT(PointCloudProperties, initialConfig, numFramesPool);

}  // namespace dai
#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for PointCloud
 */
struct PointCloudProperties : PropertiesSerializable<Properties, PointCloudProperties> {
    PointCloudConfig initialConfig;

    int numFramesPool = 4;

#if defined(__clang__)
    ~PointCloudProperties() override;
#else
    virtual ~PointCloudProperties() = default;
#endif
};

DEPTHAI_SERIALIZE_EXT(PointCloudProperties, initialConfig, numFramesPool);

}  // namespace dai
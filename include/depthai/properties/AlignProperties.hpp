#pragma once

#include "depthai/common/Interpolation.hpp"
#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/properties/FramePoolProperties.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Align
 */
struct AlignProperties : PropertiesSerializable<Properties, AlignProperties> {
    AlignConfig initialConfig;

    /// Frame pool configuration; all fields default to automatic.
    FramePoolProperties pool;

    /// Image interpolation. AUTO preserves the existing backend default.
    Interpolation interpolation = Interpolation::AUTO;

    ~AlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT(AlignProperties, initialConfig, pool, interpolation);

}  // namespace dai

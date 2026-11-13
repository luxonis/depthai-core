#pragma once

#include "depthai/common/Interpolation.hpp"
#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Align
 */
struct AlignProperties : PropertiesSerializable<Properties, AlignProperties> {
    AlignConfig initialConfig;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Image interpolation. AUTO preserves the existing backend default.
    Interpolation interpolation = Interpolation::AUTO;

    ~AlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(AlignProperties, initialConfig, numFramesPool, interpolation);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::AlignProperties>;

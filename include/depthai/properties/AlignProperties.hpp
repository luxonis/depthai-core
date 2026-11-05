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

    /// Deprecated frame count fallback for pool.numFramesPool; zero means automatic.
    [[deprecated("Use pool.numFramesPool instead")]] int numFramesPool = 0;

    /// Image interpolation. AUTO preserves the existing backend default.
    Interpolation interpolation = Interpolation::AUTO;

    /// Pool configuration. Nonzero fields override legacy fields; zero falls back to the legacy value.
    /// Legacy fields also default to zero (automatic). Prefer pool for new code.
    FramePoolProperties pool{};

    ~AlignProperties() override;
};

DEPTHAI_SERIALIZE_EXT(AlignProperties, initialConfig, numFramesPool, interpolation, pool);

}  // namespace dai

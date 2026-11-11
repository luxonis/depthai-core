#pragma once

#include "depthai/common/Backend.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct RectificationProperties : PropertiesSerializable<Properties, RectificationProperties> {
    using Backend = dai::Backend;

    std::optional<uint32_t> outputWidth, outputHeight;
    bool enableRectification = true;
    Backend backend = Backend::AUTO;
    /** Optional undistortion scaling. Unset preserves the input camera intrinsics. */
    std::optional<float> alphaScaling = std::nullopt;
    ~RectificationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RectificationProperties, outputWidth, outputHeight, enableRectification, backend, alphaScaling);

}  // namespace dai

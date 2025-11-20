#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct RectificationProperties : PropertiesSerializable<Properties, RectificationProperties> {
    std::optional<uint32_t> outputWidth, outputHeight;
    bool enableRectification = true;
    ~RectificationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RectificationProperties, outputWidth, outputHeight, enableRectification);

}  // namespace dai

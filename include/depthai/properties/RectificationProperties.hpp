#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *
 */
struct RectificationProperties : PropertiesSerializable<Properties, RectificationProperties> {
    std::optional<uint32_t> outputWidth, outputHeight;

    ~RectificationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RectificationProperties, outputWidth, outputHeight);

}  // namespace dai

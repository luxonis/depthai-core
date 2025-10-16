#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *
 */
struct RectificationProperties : PropertiesSerializable<Properties, RectificationProperties> {
    /**
     *
     */
    bool enableSync = true;

    ~RectificationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RectificationProperties, enableSync);

}  // namespace dai

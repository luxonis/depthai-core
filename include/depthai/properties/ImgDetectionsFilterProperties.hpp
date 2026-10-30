#pragma once
#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
/** Properties for ImgDetectionsFilter. */
struct ImgDetectionsFilterProperties : PropertiesSerializable<Properties, ImgDetectionsFilterProperties> {
    /** Configuration used until a message is received on inputConfig. */
    ImgDetectionsFilterConfig initialConfig = {};
    ~ImgDetectionsFilterProperties() override;
};
DEPTHAI_SERIALIZE_EXT(ImgDetectionsFilterProperties, initialConfig);
}  // namespace dai

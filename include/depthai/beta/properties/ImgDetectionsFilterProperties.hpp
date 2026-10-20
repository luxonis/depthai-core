#pragma once

#include "depthai/beta/datatype/ImgDetectionsFilterConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for ImgDetectionsFilter.
 */
struct ImgDetectionsFilterProperties : PropertiesSerializable<Properties, ImgDetectionsFilterProperties> {
    /**
     * Configuration used until a message is received on inputConfig.
     */
    ImgDetectionsFilterConfig initialConfig;

    ~ImgDetectionsFilterProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ImgDetectionsFilterProperties, initialConfig);

}  // namespace beta
}  // namespace dai

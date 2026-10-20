#pragma once

#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {
namespace beta {

/**
 * Properties for ImageOutputParser.
 */
struct ImageOutputParserProperties : PropertiesSerializable<Properties, ImageOutputParserProperties> {
    std::string outputLayerName;
    bool outputIsBGR = false;

    ~ImageOutputParserProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ImageOutputParserProperties, outputLayerName, outputIsBGR);

}  // namespace beta
}  // namespace dai

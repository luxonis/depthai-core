#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Cast
 */
struct CastProperties : PropertiesSerializable<Properties, CastProperties> {
    dai::ImgFrame::Type outputType = dai::ImgFrame::Type::RAW8;
    std::optional<float> scale;
    std::optional<float> offset;
    int numFramesPool = 4;

    ~CastProperties() override;
};

DEPTHAI_SERIALIZE_EXT(CastProperties, numFramesPool, outputType, scale, offset);

}  // namespace dai

#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct OverlayProperties : PropertiesSerializable<Properties, OverlayProperties> {
   public:
    float alpha = 0.5f;
    int interpolationType = 1;  // cv::INTER_LINEAR
};

DEPTHAI_SERIALIZE_EXT(OverlayProperties, alpha, interpolationType);

}  // namespace dai

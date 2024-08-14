#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ImageManip
 */
struct AudioMixerProperties: PropertiesSerializable<Properties, AudioMixerProperties> {
    static constexpr int AUTO = -1;

    int dummy;
};

DEPTHAI_SERIALIZE_EXT(AudioMixerProperties, dummy);

}  // namespace dai

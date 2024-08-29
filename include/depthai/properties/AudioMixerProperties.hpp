#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {
/**
 * Specify properties for ImageManip
 */
struct AudioMixerProperties : PropertiesSerializable<Properties, AudioMixerProperties> {
    static constexpr int AUTO = -1;

    bool ready = false;
};

DEPTHAI_SERIALIZE_EXT(AudioMixerProperties, ready);

}  // namespace dai

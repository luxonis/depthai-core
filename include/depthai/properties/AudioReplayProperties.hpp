#pragma once

#include <alsa/asoundlib.h>

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for AudioReplay such as audio device, ...
 */
struct AudioReplayProperties : PropertiesSerializable<Properties, AudioReplayProperties> {
    static constexpr int AUTO = -1;

    int dummy
};

DEPTHAI_SERIALIZE_EXT(AudioReplayProperties, dummy);

}  // namespace dai

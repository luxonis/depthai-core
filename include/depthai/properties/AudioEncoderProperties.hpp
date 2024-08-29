#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for AudioEncoder such as profile, bitrate, ...
 */
struct AudioEncoderProperties : PropertiesSerializable<Properties, AudioEncoderProperties> {
    static constexpr int AUTO = -1;
    /**
     *  Bitrate for the output
     */
    unsigned int bitrate = 44100;

    /**
     *  Number of audio channels (1 = mono, 2 = stereo)
     */
    unsigned int channels = 2;

    int format;

};

DEPTHAI_SERIALIZE_EXT(AudioEncoderProperties,
		      bitrate,
		      channels,
		      format);

}  // namespace dai

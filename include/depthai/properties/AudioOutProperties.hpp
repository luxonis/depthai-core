#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for AudioOut such as audio device, ...
 */
struct AudioOutProperties : PropertiesSerializable<Properties, AudioOutProperties> {
    static constexpr int AUTO = -1;

    /**
     *  Human readable name for the device
     */
    std::string audioOutName = "";

    /**
     *  ALSA name for the device
     */
    std::string audioOutPath = "hw:0";

    /**
     *  Bitrate for the output
     */
    unsigned int bitrate = 44100;

    /**
     *  Number of buffered audio frames before sending over
     *  More frames = longer delay, but less load
     */
    unsigned int framesPerSecond = 10;

    /**
     *  Number of audio channels (1 = mono, 2 = stereo)
     */
    unsigned int channels = 2;

};

DEPTHAI_SERIALIZE_EXT(AudioOutProperties,
		      audioOutName,
		      audioOutPath,
		      bitrate,
		      framesPerSecond,
		      channels);

}  // namespace dai

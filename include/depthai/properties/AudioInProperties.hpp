#pragma once

#include "depthai/properties/Properties.hpp"

#include <alsa/asoundlib.h>

namespace dai {

/**
 *  Specify properties for AudioIn such as audio device, ...
 */
struct AudioInProperties : PropertiesSerializable<Properties, AudioInProperties> {
    static constexpr int AUTO = -1;

    /**
     *  Human readable name for the device
     */
    std::string audioInName = "";

    /**
     *  ALSA name for the device
     */
    std::string audioInPath = "hw:0";

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
    
    snd_pcm_format_t format;

};

DEPTHAI_SERIALIZE_EXT(AudioInProperties,
		      audioInName,
		      audioInPath,
		      bitrate,
		      framesPerSecond,
		      channels,
		      format);

}  // namespace dai

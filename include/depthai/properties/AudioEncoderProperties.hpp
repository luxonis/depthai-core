#pragma once

#include "depthai/properties/Properties.hpp"

namespace dai {

enum AudioEncoderFormat {
	AUDIO_ENCODER_RAW,
	AUDIO_ENCODER_WAV,
};


/**
 * Specify properties for VideoEncoder such as profile, bitrate, ...
 */
struct AudioEncoderProperties : PropertiesSerializable<Properties, AudioEncoderProperties> {
    static constexpr int AUTO = -1;

    /**
     *  Input 
     */
    AudioEncoderFormat inputEncoderFormat;
    
    /**
     *  Output
     */
    AudioEncoderFormat outputEncoderFormat;

    /**
     *  Number of buffered audio frames before encoding
     *  More frames = longer delay, but less load
     */
    unsigned int framesPerSecond = 10;

    unsigned int frameSize;
    /**
     *  Bitrate for the output
     */
    unsigned int bitrate = 44100;


    /**
     *  Number of audio channels (1 = mono, 2 = stereo)
     */
    unsigned int channels = 2;

};

DEPTHAI_SERIALIZE_EXT(AudioEncoderProperties,
		      inputEncoderFormat,
		      outputEncoderFormat,
		      framesPerSecond,
		      frameSize,
		      bitrate,
		      channels);

}  // namespace dai

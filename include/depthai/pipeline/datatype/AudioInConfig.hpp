#pragma once

#include "depthai-shared/datatype/RawAudioInConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AudioInConfig message. Carries audio input (microphone) config.
 */
class AudioInConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAudioInConfig& cfg;

   public:
    /**
     * Construct AudioInConfig message.
     */
    AudioInConfig();
    explicit AudioInConfig(std::shared_ptr<RawAudioInConfig> ptr);
    virtual ~AudioInConfig() = default;

    /// Set a fixed microphone gain, in multiplication times
    void setMicGainTimes(float times);

    /// Set a fixed microphone gain, in dB
    void setMicGainDecibels(float dB);

    /// Set passthrough flag
    void setPassthrough(bool passThrough);

    /// Set disable output flag
    void setDisableOutput(bool disable);

    /**
     * Retrieve configuration data for AudioIn
     * @returns AudioInConfigData: mic gain, ...
     */
    AudioInConfigData getConfigData() const;
};

}  // namespace dai

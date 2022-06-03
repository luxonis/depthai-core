#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/AudioMicProperties.hpp>

#include "depthai/pipeline/datatype/AudioInConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief AudioMic (microphone) node
 */
class AudioMic : public NodeCRTP<Node, AudioMic, AudioMicProperties>  {
   public:
    constexpr static const char* NAME = "AudioMic";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawAudioInConfig> rawConfig;

   public:
    AudioMic(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    AudioMic(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use for edge detection.
     */
    AudioInConfig initialConfig;

    /**
     * Input AudioInConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::AudioInConfig, false}}};

    /**
     * Outputs audio data from onboard microphones. Reusing ImgFrame for now
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output outBack{*this, "outBack", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /// Enable streaming back microphone instead of the front ones (L/R)
    void setStreamBackMic(bool enable);

    /// Enable experimental digital AGC
    void setMicAutoGain(bool enable);

    /// Apply mic gain to XLink output as well. Enabled by default
    void setXlinkApplyMicGain(bool enable);

    /// XLink sample size in bytes. Default 3, other options: 2 or 4
    void setXlinkSampleSizeBytes(int size);
};

}  // namespace node
}  // namespace dai

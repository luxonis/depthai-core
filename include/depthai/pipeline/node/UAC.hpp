#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UACProperties.hpp>

namespace dai {
namespace node {
class UAC : public Node {
    dai::UACProperties properties;

    std::string getName() const override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Outputs audio data from onboard microphones. Reusing ImgFrame for now
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /// Enable streaming back microphone instead of the front ones (L/R)
    void setStreamBackMic(bool enable);

    /// Enable experimental digital AGC
    void setMicAutoGain(bool enable);

    /// Set a fixed microphone gain, in multiplication times
    void setMicGainTimes(float times);

    /// Set a fixed microphone gain, in dB
    void setMicGainDecibels(float dB);

    /// Apply mic gain to XLink output as well. Enabled by default
    void setXlinkApplyMicGain(bool enable);

    /// XLink sample size in bytes. Default 3, other options: 2 or 4
    void setXlinkSampleSizeBytes(int size);

    /// Enable speaker
    void setEnableSpeaker(bool enable);

    /// Set speaker volume, 0..100. Default: 70
    void setSpeakerVolume(int volume);

};

}  // namespace node
}  // namespace dai

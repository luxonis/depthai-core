#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UACProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief UAC (USB Audio Class) node
 */
class UAC : public NodeCRTP<Node, UAC, UACProperties>  {
   public:
    constexpr static const char* NAME = "UAC";

   public:
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

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
};

}  // namespace node
}  // namespace dai

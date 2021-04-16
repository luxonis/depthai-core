#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UACProperties.hpp>

namespace dai {
namespace node {
class UAC : public Node {
    dai::UACProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /// Enable streaming back microphone instead of the front ones (L/R)
    void setStreamBackMic(bool enable);

    /// Enable experimental digital AGC
    void setMicAutoGain(bool enable);

    /// Set a fixed microphone gain, in multiplication times
    void setMicGainTimes(float times);

    /// Set a fixed microphone gain, in dB
    void setMicGainDecibels(float dB);
};

}  // namespace node
}  // namespace dai

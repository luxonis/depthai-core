#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/AudioFrame.hpp"
#include "depthai/properties/AudioEncoderProperties.hpp"
#include "depthai/utility/AudioHelpers.hpp"

namespace dai {
namespace node {

class AudioEncoder : public DeviceNodeCRTP<DeviceNode, AudioEncoder, AudioEncoderProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:  // internal usage
    constexpr static const char* NAME = "AudioEncoder";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioEncoder() = default;
    AudioEncoder(std::unique_ptr<Properties> props);

    std::shared_ptr<AudioEncoder> build() {
        return std::static_pointer_cast<AudioEncoder>(shared_from_this());
    }

    std::vector<float> convertToFloat(std::shared_ptr<AudioFrame> input);
    std::vector<float> resampleAudio(std::vector<float> input, int inputSampleRate, int outputSampleRate, int channels);
    std::shared_ptr<AudioFrame> convertFromFloat(std::vector<float> input, std::shared_ptr<AudioFrame> output);

    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::AudioFrame, true}}}}};
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::AudioFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    unsigned int getBitrate() const;
    unsigned int getChannels() const;
    int getFormat() const;

    void setBitrate(unsigned int bitrate);
    void setChannels(unsigned int channels);
    void setFormat(int format);

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;
};

}  // namespace node
}  // namespace dai

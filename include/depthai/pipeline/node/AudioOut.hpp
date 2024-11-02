#pragma once

// depthai
#include <alsa/asoundlib.h>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/AudioFrame.hpp"
#include "depthai/properties/AudioOutProperties.hpp"

namespace dai {
namespace node {

class AudioOut : public DeviceNodeCRTP<DeviceNode, AudioOut, AudioOutProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:  // internal usage
    constexpr static const char* NAME = "AudioOut";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioOut() = default;
    AudioOut(std::unique_ptr<Properties> props);
    std::shared_ptr<AudioOut> build() {
        return std::static_pointer_cast<AudioOut>(shared_from_this());
    }

    void setDeviceName(std::string audioInName);
    void setDevicePath(std::string audioInPath);
    void setBitrate(unsigned int bitrate);
    void setFps(unsigned int fps);
    void setChannels(unsigned int channels);
    void setFormat(int format);

    std::string getDeviceName() const;
    std::string getDevicePath() const;
    unsigned int getBitrate() const;
    unsigned int getFps() const;
    unsigned int getChannels() const;
    int getFormat() const;

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

    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::AudioFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
};

}  // namespace node
}  // namespace dai

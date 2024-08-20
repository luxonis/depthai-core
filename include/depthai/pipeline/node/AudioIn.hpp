#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioInProperties.hpp"

#include <alsa/asoundlib.h>

namespace dai {
namespace node {

class AudioIn: public DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;
    snd_pcm_t *captureHandle;
   public:  // internal usage
    constexpr static const char* NAME = "AudioIn";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioIn() = default;
    AudioIn(std::unique_ptr<Properties> props);

    ~AudioIn();

    std::shared_ptr<AudioIn> build();

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

   Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioOutProperties.hpp"

namespace dai {
namespace node {

class AudioOut: public DeviceNodeCRTP<DeviceNode, AudioOut, AudioOutProperties>, public SourceNode { 
   public:  // internal usage
    constexpr static const char* NAME = "AudioOut";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioOut() = default;
    AudioOut(std::unique_ptr<Properties> props);

    ~AudioOut();

    std::shared_ptr<AudioOut> build();

    void setDeviceName(std::string audioInName);
    void setDevicePath(std::string audioInPath);
    void setBitrate(unsigned int bitrate);
    void setFps(unsigned int fps);
    void setChannels(unsigned int channels);

    std::string getDeviceName();
    std::string getDevicePath();
    unsigned int getBitrate();
    unsigned int getFps();
    unsigned int getChannels();

    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

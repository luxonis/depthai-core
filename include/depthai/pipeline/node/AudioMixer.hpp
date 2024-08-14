#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioMixerProperties.hpp"

namespace dai {
namespace node {

class AudioMixer: public DeviceNodeCRTP<DeviceNode, AudioMixer, AudioMixerProperties>, public SourceNode { 
   public:  // internal usage
    constexpr static const char* NAME = "AudioMixer";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioMixer() = default;
    AudioMixer(std::unique_ptr<Properties> props);

    ~AudioMixer();

    std::shared_ptr<AudioMixer> build();

    OutputMap outputs{*this, "outputs", {DEFAULT_NAME, DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    InputMap inputs{*this, "inputs", {DEFAULT_NAME, DEFAULT_GROUP, false, 1, {{{DatatypeEnum::Buffer, true}}}, true}};

   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

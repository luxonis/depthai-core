#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioEncoderProperties.hpp"

namespace dai {
namespace node {

class AudioEncoder: public DeviceNodeCRTP<DeviceNode, AudioEncoder, AudioEncoderProperties>, public SourceNode { 
   public:  // internal usage
    constexpr static const char* NAME = "AudioEncoder";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioEncoder() = default;
    AudioEncoder(std::unique_ptr<Properties> props);

    ~AudioEncoder();

    std::shared_ptr<AudioEncoder> build();

    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

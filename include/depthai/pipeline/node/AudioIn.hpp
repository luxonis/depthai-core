#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioInProperties.hpp"

namespace dai {
namespace node {

class AudioIn: public DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>, public SourceNode { 
   public:  // internal usage
    constexpr static const char* NAME = "AudioIn";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioIn() = default;
    AudioIn(std::unique_ptr<Properties> props);

    ~AudioIn();

    std::shared_ptr<AudioIn> build();

   Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

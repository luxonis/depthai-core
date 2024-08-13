#pragma once

// depthai
//#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
//#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/properties/AudioInProperties.hpp"
//#include "depthai/utility/span.hpp"

#include <alsa/asoundlib.h>
namespace dai {
namespace node {

class AudioIn: public DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>, public SourceNode {
   public:
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

   public:  // internal usage
    constexpr static const char* NAME = "AudioIn";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioIn() = default;
    AudioIn(std::unique_ptr<Properties> props);

    ~AudioIn();

    std::shared_ptr<AudioIn> build();
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

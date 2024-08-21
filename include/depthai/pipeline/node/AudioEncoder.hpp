#pragma once

// depthai
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/AudioEncoderProperties.hpp"

namespace dai {
namespace node {

class AudioEncoder: public DeviceNodeCRTP<DeviceNode, AudioEncoder, AudioEncoderProperties>, public HostRunnable { 
   private:
    bool runOnHostVar = false;
   public:  // internal usage
    constexpr static const char* NAME = "AudioEncoder";

    using DeviceNodeCRTP::DeviceNodeCRTP;
    AudioEncoder() = default;
    AudioEncoder(std::unique_ptr<Properties> props);

    ~AudioEncoder();

    std::shared_ptr<AudioEncoder> build();

    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

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
   protected:
    bool isBuild = false;
    bool needsBuild() override {
        return !isBuild;
    }
};

}  // namespace node
}  // namespace dai

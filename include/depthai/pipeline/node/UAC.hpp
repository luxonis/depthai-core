#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UACProperties.hpp>

#include "depthai/pipeline/datatype/AudioInConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief UAC (USB Audio Class) node
 */
class UAC : public NodeCRTP<Node, UAC, UACProperties>  {
   public:
    constexpr static const char* NAME = "UAC";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawAudioInConfig> rawConfig;

   public:
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use for audio config - TODO
     */
    AudioInConfig initialConfig;

    /**
     * Input AudioInConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::AudioInConfig, false}}};

    /**
     * Input for audio data to be streamed over UAC. Reusing ImgFrame for now.
     * Default queue is non-blocking with size 4.
     */
    Input input{*this, "input", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};
};

}  // namespace node
}  // namespace dai

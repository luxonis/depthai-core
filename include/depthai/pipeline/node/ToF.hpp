#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/ToFProperties.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ToF node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class ToF : public NodeCRTP<DeviceNode, ToF, ToFProperties> {
   public:
    constexpr static const char* NAME = "ToF";
    using NodeCRTP::NodeCRTP;

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawToFConfig> rawConfig;

   public:
    ToF();
    ToF(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    ToFConfig initialConfig;

    /**
     * Input ToFConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ToFConfig, false}}};
    /**
     * Input message with frame data on which feature tracking is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputRaw{true, *this, "inputRaw", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    Output depth{true, *this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    // TODO(before mainline) - API not supported on RVC3
    Input input{true, *this, "input", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInputRaw{true, *this, "passthroughInputRaw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    // TODO(before mainline) - API not supported on RVC3
    Output amplitude{true, *this, "amplitude", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output intensity{true, *this, "intensity", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output error{true, *this, "error", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
};

}  // namespace node
}  // namespace dai

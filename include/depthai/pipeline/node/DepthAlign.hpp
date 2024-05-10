#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DepthAlignProperties.hpp>

#include "depthai/pipeline/datatype/DepthAlignConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief DepthAlign node. Calculates spatial location data on a set of ROIs on depth map.
 */
class DepthAlign : public NodeCRTP<Node, DepthAlign, DepthAlignProperties> {
   public:
    constexpr static const char* NAME = "DepthAlign";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawDepthAlignConfig> rawConfig;

   public:
    DepthAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    DepthAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    DepthAlignConfig initialConfig;

    /**
     * Input message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::DepthAlignConfig, false}}};

    /**
     * Input message.
     * Default queue is non-blocking with size 4.
     */
    Input input{*this, "input", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input align to message.
     * Default queue is non-blocking with size 1.
     */
    Input inputAlignTo{*this, "inputAlignTo", Input::Type::SReceiver, false, 1, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that is aligned to inputAlignTo.
     */
    Output outputAligned{*this, "outputAligned", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInput{*this, "passthroughInput", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify the output size of the aligned depth map
     */
    DepthAlign& setOutputSize(int alignWidth, int alignHeight);
};

}  // namespace node
}  // namespace dai
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
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::DepthAlignConfig, false}}};
    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries aligned depth image.
     */
    Output outputAlignedDepth{*this, "outputAlignedDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

};

}  // namespace node
}  // namespace dai
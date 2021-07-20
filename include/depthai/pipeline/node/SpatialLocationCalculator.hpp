#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SpatialLocationCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief SpatialLocationCalculator node. Calculates spatial location data on a set of ROIs on depth map.
 */
class SpatialLocationCalculator : public Node {
   public:
    using Properties = dai::SpatialLocationCalculatorProperties;

   private:
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawSpatialLocationCalculatorConfig> rawConfig;
    Properties properties;

   public:
    std::string getName() const override;

    SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Initial config to use when calculating spatial location data.
     */
    SpatialLocationCalculatorConfig initialConfig;

    /**
     * Input SpatialLocationCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};
    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs SpatialLocationCalculatorData message that carries spatial location results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorData, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    void setWaitForConfigInput(bool wait);
};

}  // namespace node
}  // namespace dai

#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SpatialLocationCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace node {
class SpatialLocationCalculator : public Node {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawSpatialLocationCalculatorConfig> rawConfig;

   public:
    SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    SpatialLocationCalculatorConfig initialConfig;

    dai::SpatialLocationCalculatorProperties properties;

    void setWaitForConfigInput(bool wait);

    /**
     * Input SpatialLocationCalculator message with ability to modify parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 8, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, false}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorData, false}}};
};

}  // namespace node
}  // namespace dai

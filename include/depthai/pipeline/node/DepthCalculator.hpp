#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DepthCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/DepthCalculatorConfig.hpp"

namespace dai {
namespace node {
class DepthCalculator : public Node {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    std::shared_ptr<RawDepthCalculatorConfig> rawConfig;

   public:
    DepthCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    DepthCalculatorConfig initialConfig;

    dai::DepthCalculatorProperties properties;

    void setWaitForConfigInput(bool wait);

    /**
     * Input DepthCalculator message with ability to modify parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 8, {{DatatypeEnum::DepthCalculatorConfig, false}}};
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, false}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::DepthCalculatorData, false}}};
};

}  // namespace node
}  // namespace dai

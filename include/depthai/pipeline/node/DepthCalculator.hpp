#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DepthCalculatorProperties.hpp>

namespace dai {
namespace node {
class DepthCalculator : public Node {
    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    DepthCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    dai::DepthCalculatorProperties properties;

    void setROIs(std::vector<DepthCalculatorConfig> rois);
    void addROI(DepthCalculatorConfig& roi);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
    Input depthInput{*this, "inDepth", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, false}}};

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::DepthCalculatorData, false}}};
};

}  // namespace node
}  // namespace dai

#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UVCProperties.hpp>

namespace dai {
namespace node {
class UVC : public Node {
    dai::UVCProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};

    /// Set GPIO list <gpio_number, value> for GPIOs to set (on/off) at init
    void setGpiosOnInit(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is enabled
    void setGpiosOnStreamOn(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is disabled
    void setGpiosOnStreamOff(std::unordered_map<int, int> list);
};

}  // namespace node
}  // namespace dai

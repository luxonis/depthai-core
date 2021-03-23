#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
// TODO #include <depthai-shared/pb/properties/XLinkOutProperties.hpp>

namespace dai {
namespace node {
class UVC : public Node {
    // dai::XLinkOutProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
};

}  // namespace node
}  // namespace dai

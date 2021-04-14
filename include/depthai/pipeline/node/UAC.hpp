#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
// TODO #include <depthai-shared/pb/properties/XLinkOutProperties.hpp>

namespace dai {
namespace node {
class UAC : public Node {
    // dai::XLinkOutProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    UAC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

};

}  // namespace node
}  // namespace dai

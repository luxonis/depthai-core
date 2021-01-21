#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/pb/properties/SystemLoggerProperties.hpp>

namespace dai {
namespace node {
class SystemLogger : public Node {
    dai::SystemLoggerProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SystemInformation, false}}};

    void setRate(float hz);
};

}  // namespace node
}  // namespace dai

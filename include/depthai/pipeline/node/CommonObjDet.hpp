#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/pb/properties/CommonObjDetProperties.hpp>

namespace dai {
namespace node {
class CommonObjDet : public Node {
    dai::CommonObjDetProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    CommonObjDet(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Input input{*this, "in", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, false}}};

    void setStreamName(const std::string& name);
    void setNNConfigPath(const std::string& path);
};

}  // namespace node
}  // namespace dai

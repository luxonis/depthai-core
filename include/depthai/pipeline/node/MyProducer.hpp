#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/MyProducerProperties.hpp>

namespace dai {
namespace node {
class MyProducer : public Node {
    dai::MyProducerProperties properties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    MyProducer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    void setMessage(const std::string& m);
    void setProcessor(ProcessorType proc);
};

}  // namespace node
}  // namespace dai

#include "depthai/pipeline/node/MyProducer.hpp"

namespace dai {
namespace node {

MyProducer::MyProducer(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    outputs = {&out};
}

std::string MyProducer::getName() const {
    return "MyProducer";
}

nlohmann::json MyProducer::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> MyProducer::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void MyProducer::setMessage(const std::string& m) {
    properties.message = m;
}

void MyProducer::setProcessor(ProcessorType proc) {
    properties.processorPlacement = proc;
}

}  // namespace node
}  // namespace dai

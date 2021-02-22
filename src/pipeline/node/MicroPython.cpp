#include "depthai/pipeline/node/MicroPython.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

MicroPython::MicroPython(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string MicroPython::getName() const {
    return "MicroPython";
}

std::vector<Node::Output> MicroPython::getOutputs() {
    return {};
}

std::vector<Node::Input> MicroPython::getInputs() {
    return {input};
}

nlohmann::json MicroPython::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> MicroPython::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void MicroPython::setScriptPath(const std::string& path) {
    auto asset = loadAsset("blob", path);
    properties.scriptUri = asset->getUri();
    properties.scriptSize = asset->data.size();
}

}  // namespace node
}  // namespace dai

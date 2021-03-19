#include "depthai/pipeline/node/MicroPython.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

MicroPython::MicroPython(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId),
      inputs(Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}})),
      outputs(Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})) {
    properties.scriptUri = "";
    properties.processor = ProcessorType::LEON_MSS;
}

std::string MicroPython::getName() const {
    return "MicroPython";
}

std::vector<Node::Output> MicroPython::getOutputs() {
    std::vector<Node::Output> vecOutputs;
    for(const auto& kv : outputs) {
        vecOutputs.push_back(kv.second);
    }
    return vecOutputs;
}

std::vector<Node::Input> MicroPython::getInputs() {
    std::vector<Node::Input> vecInputs;
    for(const auto& kv : inputs) {
        vecInputs.push_back(kv.second);
    }
    return vecInputs;
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
    scriptPath = path;
}

void MicroPython::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

std::string MicroPython::getScriptPath() const {
    return scriptPath;
}

ProcessorType MicroPython::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai

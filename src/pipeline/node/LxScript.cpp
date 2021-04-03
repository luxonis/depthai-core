#include "depthai/pipeline/node/LxScript.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

LxScript::LxScript(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId),
      inputs(Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}})),
      outputs(Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})) {
    properties.scriptUri = "";
    properties.processor = ProcessorType::LEON_MSS;
}

std::string LxScript::getName() const {
    return "LxScript";
}

std::vector<Node::Output> LxScript::getOutputs() {
    std::vector<Node::Output> vecOutputs;
    for(const auto& kv : outputs) {
        vecOutputs.push_back(kv.second);
    }
    return vecOutputs;
}

std::vector<Node::Input> LxScript::getInputs() {
    std::vector<Node::Input> vecInputs;
    for(const auto& kv : inputs) {
        vecInputs.push_back(kv.second);
    }
    return vecInputs;
}

nlohmann::json LxScript::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> LxScript::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void LxScript::setName(const std::string& name){
    properties.nodeName = name;
}

void LxScript::setScriptPath(const std::string& path) {
    properties.scriptUri = assetManager.add("__script", path)->getRelativeUri();
    scriptPath = path;
}

void LxScript::setScriptData(const std::string& script) {
    std::vector<std::uint8_t> data{script.begin(), script.end()};
    properties.scriptUri = assetManager.add("__script", std::move(data))->getRelativeUri();
    scriptPath = "";
}

void LxScript::setScriptData(const std::vector<std::uint8_t>& data){
    properties.scriptUri = assetManager.add("__script", std::move(data))->getRelativeUri();
    scriptPath = "";
}

void LxScript::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

std::string LxScript::getScriptPath() const {
    return scriptPath;
}

ProcessorType LxScript::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai

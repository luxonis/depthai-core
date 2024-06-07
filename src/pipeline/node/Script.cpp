#include "depthai/pipeline/node/Script.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

std::shared_ptr<Script> Script::build() {
    // Set some default properties
    properties.scriptUri = "";
    properties.scriptName = "<script>";
    properties.processor = ProcessorType::LEON_MSS;

    isBuild = true;
    return std::static_pointer_cast<Script>(shared_from_this());
}

void Script::setScriptPath(const dai::Path& path, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", path)->getRelativeUri();
    scriptPath = path;
    if(name.empty()) {
        properties.scriptName = path.u8string();
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::string& script, const std::string& name) {
    std::vector<std::uint8_t> data{script.begin(), script.end()};
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = {};
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::vector<std::uint8_t>& data, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = {};
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

dai::Path Script::getScriptPath() const {
    return scriptPath;
}

std::string Script::getScriptName() const {
    return properties.scriptName;
}

ProcessorType Script::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai

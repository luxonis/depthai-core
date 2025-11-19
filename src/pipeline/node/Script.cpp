#include "depthai/pipeline/node/Script.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

void Script::buildInternal() {}

void Script::setScriptPath(const std::filesystem::path& path, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", path)->getRelativeUri();
    scriptPath = path;
    if(name.empty()) {
        properties.scriptName = path.string();
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::string& script, const std::string& name) {
    std::vector<std::uint8_t> data{script.begin(), script.end()};
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = std::filesystem::path();
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::vector<std::uint8_t>& data, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = std::filesystem::path();
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

std::filesystem::path Script::getScriptPath() const {
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

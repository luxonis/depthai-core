#include "depthai/pipeline/node/Script.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

namespace {

std::string pathToUtf8String(const std::filesystem::path& path) {
#ifdef _WIN32
    const auto utf8 = path.u8string();
    return {reinterpret_cast<const char*>(utf8.data()), utf8.size()};
#else
    return path.string();
#endif
}

}  // namespace

void Script::buildInternal() {}

void Script::buildStage1() {
    if(properties.scriptUri.empty()) {
        throw std::runtime_error("No script set. Please set a script using setScriptPath or setScript.");
    }
}

void Script::setScriptPath(const std::filesystem::path& path, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", path)->getRelativeUri();
    scriptPath = path;
    if(name.empty()) {
        properties.scriptName = pathToUtf8String(path);
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

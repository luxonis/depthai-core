#include "depthai/pipeline/node/CommonObjDet.hpp"

#include <sstream>

namespace dai {
namespace node {

    CommonObjDet::CommonObjDet(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {
    }

    std::string CommonObjDet::getName() {
        return "CommonObjDet";
    }

    std::vector<Node::Input> CommonObjDet::getInputs() {
        return {input};
    }

    std::vector<Node::Output> CommonObjDet::getOutputs() {
        return {out};
    }

    nlohmann::json CommonObjDet::getProperties() {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> CommonObjDet::clone() {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

    void CommonObjDet::setStreamName(const std::string& name) {
        properties.streamName = name;
    }

    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    void CommonObjDet::setNNConfigPath(const std::string& path) {
        std::ifstream configStream(path);
        std::stringstream configBuffer;
        configBuffer << configStream.rdbuf();
        properties.nnConfig = configBuffer.str();
    }

}  // namespace node
}  // namespace dai

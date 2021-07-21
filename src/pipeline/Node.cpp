#include "depthai/pipeline/Node.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {

Node::Node(const std::shared_ptr<PipelineImpl>& p, Id nodeId) : parent(p), id(nodeId) {}

tl::optional<OpenVINO::Version> Node::getRequiredOpenVINOVersion() {
    return tl::nullopt;
}

const Pipeline Node::getParentPipeline() const {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Pipeline Node::getParentPipeline() {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Node::Connection::Connection(Output out, Input in) {
    outputId = out.getParent().id;
    outputName = out.name;
    inputId = in.getParent().id;
    inputName = in.name;
}

bool Node::Connection::operator==(const Node::Connection& rhs) const {
    return (outputId == rhs.outputId && outputName == rhs.outputName && inputId == rhs.inputId && inputName == rhs.inputName);
}

std::vector<Node::Connection> Node::Output::getConnections() {
    std::vector<Node::Connection> myConnections;
    auto allConnections = parent.getParentPipeline().getConnections();
    for(const auto& conn : allConnections) {
        if(conn.outputId == parent.id && conn.outputName == name) {
            myConnections.push_back(conn);
        }
    }
    return myConnections;
}

bool Node::Output::isSamePipeline(const Input& in) {
    // Check whether current output and 'in' are on same pipeline.
    // By checking parent of node
    auto outputPipeline = parent.parent.lock();
    if(outputPipeline != nullptr) {
        return (outputPipeline == in.parent.parent.lock());
    }
    return false;
}

bool Node::Output::canConnect(const Input& in) {
    return PipelineImpl::canConnect(*this, in);
}

void Node::Output::link(const Input& in) {
    // Call link of pipeline
    parent.getParentPipeline().link(*this, in);
}

void Node::Output::unlink(const Input& in) {
    // Call unlink of pipeline parents pipeline
    parent.getParentPipeline().unlink(*this, in);
}

void Node::Input::setBlocking(bool blocking) {
    this->blocking = blocking;
}

bool Node::Input::getBlocking() const {
    if(blocking) {
        return *blocking;
    }
    return defaultBlocking;
}

void Node::Input::setQueueSize(int size) {
    this->queueSize = size;
}

int Node::Input::getQueueSize() const {
    if(queueSize) {
        return *queueSize;
    }
    return defaultQueueSize;
}

const AssetManager& Node::getAssetManager() const {
    return assetManager;
}

AssetManager& Node::getAssetManager() {
    return assetManager;
}

Node::OutputMap::OutputMap(Node::Output defaultOutput) : defaultOutput(defaultOutput) {}
Node::Output& Node::OutputMap::operator[](const std::string& key) {
    if(count(key) == 0) {
        // Create using default and rename with key
        auto& d = defaultOutput;
        insert(std::make_pair(key, Output(d.getParent(), key, d.type, d.possibleDatatypes)));
    }
    // otherwise just return reference to existing
    return at(key);
}

Node::InputMap::InputMap(Node::Input defaultInput) : defaultInput(defaultInput) {}
Node::Input& Node::InputMap::operator[](const std::string& key) {
    if(count(key) == 0) {
        // Create using default and rename with key
        auto& d = defaultInput;
        insert(std::make_pair(key, Input(d.getParent(), key, d.type, d.defaultBlocking, d.defaultQueueSize, d.possibleDatatypes)));
    }
    // otherwise just return reference to existing
    return at(key);
}

/// Retrieves all nodes outputs
std::vector<Node::Output> Node::getOutputs() {
    std::vector<Node::Output> result;
    for(auto* x : getOutputRefs()) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves all nodes inputs
std::vector<Node::Input> Node::getInputs() {
    std::vector<Node::Input> result;
    for(auto* x : getInputRefs()) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves reference to node outputs
std::vector<Node::Output*> Node::getOutputRefs() {
    std::vector<Node::Output*> outputRefs{outputs.begin(), outputs.end()};
    for(auto*& map : outputMaps) {
        for(auto& kv : *map) {
            outputRefs.push_back(&kv.second);
        }
    }
    return outputRefs;
}

/// Retrieves reference to node outputs
std::vector<const Node::Output*> Node::getOutputRefs() const {
    std::vector<const Node::Output*> outputRefs{outputs.begin(), outputs.end()};
    for(const auto* const& map : outputMaps) {
        for(const auto& kv : *map) {
            outputRefs.push_back(&kv.second);
        }
    }
    return outputRefs;
}
/// Retrieves reference to node inputs
std::vector<Node::Input*> Node::getInputRefs() {
    std::vector<Node::Input*> inputRefs{inputs.begin(), inputs.end()};
    for(auto*& map : inputMaps) {
        for(auto& kv : *map) {
            inputRefs.push_back(&kv.second);
        }
    }
    return inputRefs;
}

/// Retrieves reference to node inputs
std::vector<const Node::Input*> Node::getInputRefs() const {
    std::vector<const Node::Input*> inputRefs{inputs.begin(), inputs.end()};
    for(const auto* const& map : inputMaps) {
        for(const auto& kv : *map) {
            inputRefs.push_back(&kv.second);
        }
    }
    return inputRefs;
}

}  // namespace dai

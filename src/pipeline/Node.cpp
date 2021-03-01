#include "depthai/pipeline/Node.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {

Node::Node(const std::shared_ptr<PipelineImpl>& p, Id nodeId) : parent(p), id(nodeId) {}

std::vector<std::shared_ptr<Asset>> Node::getAssets() {
    return assetManager.getAll();
}

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
    outputId = out.parent.id;
    outputName = out.name;
    inputId = in.parent.id;
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

}  // namespace dai

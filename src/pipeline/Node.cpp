#include "depthai/pipeline/Node.hpp"

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {

Node::Node(const std::shared_ptr<PipelineImpl>& p, int64_t nodeId) : parent(p), id(nodeId) {}

void Node::loadAssets(AssetManager& assetManager) {
    (void)assetManager;
}

Pipeline Node::getParentPipeline() {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

bool Node::Output::canConnect(const Input& in) {
    if(type == Type::MSender && in.type == Input::Type::MReceiver) return false;
    if(type == Type::SSender && in.type == Input::Type::SReceiver) return false;
    for(const auto& outHierarchy : possibleDatatypes) {
        for(const auto& inHierarchy : in.possibleDatatypes) {
            if(outHierarchy.datatype == inHierarchy.datatype) return true;
            if(inHierarchy.descendants && isDatatypeSubclassOf(inHierarchy.datatype, outHierarchy.datatype)) return true;
        }
    }
    return false;
}

void Node::Output::link(Input in) {
    if(!canConnect(in)) {
        std::string msg = "Cannot link '" + parent.getName() + "." + name + "' to '" + in.parent.getName() + in.name + "'";
        throw std::runtime_error(msg);
    }

    conn.push_back(in);
}

}  // namespace dai

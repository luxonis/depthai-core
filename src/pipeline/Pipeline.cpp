#include "depthai/pipeline/Pipeline.hpp"

namespace dai {

int64_t PipelineImpl::getNextUniqueId() {
    return latestId++;
}

Pipeline::Pipeline() {
    pimpl = std::make_shared<PipelineImpl>();
}

/*
Pipeline::Pipeline(const Pipeline& p){
    pimpl = std::make_shared<PipelineImpl>();

    // Copy all nodes
    pimpl->globalProperties = p.getGlobalProperties();
    pimpl->nodes.reserve(p.pimpl->nodes.size());
    for(const auto& n : p.pimpl->nodes){
        auto clone = n->clone();
        clone->parent = std::weak_ptr<PipelineImpl>(pimpl);
        pimpl->nodes.push_back(clone);
    }
}
*/

Pipeline::Pipeline(const std::shared_ptr<PipelineImpl>& pimpl) {
    this->pimpl = pimpl;
}

GlobalProperties Pipeline::getGlobalProperties() const {
    return pimpl->globalProperties;
}

/*
void Pipeline::loadAssets(AssetManager& assetManager) {
    return pimpl->loadAssets(assetManager);
}
*/

/*
void PipelineImpl::loadAssets() {

    // Load assets of nodes
    for(const auto& node : nodes){
        node->loadAssets(assetManager);
    }

    // Load assets of pipeline (if any)
    // ...

}
*/

void PipelineImpl::serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage) {
    schema = getPipelineSchema();
    assetManager.serialize(assets, assetStorage);
}

AssetManager& PipelineImpl::getAssetManager() {
    return assetManager;
}

PipelineSchema PipelineImpl::getPipelineSchema() {
    // create internal representation
    PipelineSchema schema;
    schema.globalProperties = globalProperties;

    for(const auto& node : nodes) {
        // Create 'node' info
        NodeObjInfo info;
        info.id = node->id;
        info.name = node->getName();
        info.properties = node->getProperties();
        schema.nodes.push_back(info);

        // Create 'connections' info
        // Loop through connections (output -> input)
        for(const auto& output : node->getOutputs()) {
            NodeConnectionSchema connection;
            connection.node1Id = node->id;
            connection.node1Output = output.name;

            for(const auto& input : output.conn) {
                connection.node2Id = input.parent.id;
                connection.node2Input = input.name;
                schema.connections.push_back(connection);
            }
        }
    }

    return schema;
    // end of internal representation
}

}  // namespace dai

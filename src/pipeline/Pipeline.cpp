#include "depthai/pipeline/Pipeline.hpp"

#include "depthai-shared/generated/PipelineBuilderGenerated.hpp"
#include "depthai-shared/generated/Generators.hpp"


namespace dai
{

    int64_t PipelineImpl::getNextUniqueId(){
        return latestId++;
    }

    PipelineImpl::PipelineImpl(){}

    Pipeline::Pipeline(){
        pimpl = std::shared_ptr<PipelineImpl>(new PipelineImpl());
    }

    Pipeline::Pipeline(const Pipeline& p){
        pimpl = std::shared_ptr<PipelineImpl>(new PipelineImpl());
        
        // Copy all nodes
        pimpl->globalProperties = p.getGlobalProperties();
        pimpl->nodes.reserve(p.pimpl->nodes.size());
        for(const auto& n : p.pimpl->nodes){
            auto clone = n->clone();
            clone->parent = std::weak_ptr<PipelineImpl>(pimpl);
            pimpl->nodes.push_back(clone);
        }
    }

    std::string Pipeline::toJson(){
        return pimpl->toJson();
    }

    gen::GlobalProperties Pipeline::getGlobalProperties() const {
        return pimpl->globalProperties;
    }
    
    std::string PipelineImpl::toJson(){

        // create internal representation
        gen::PipelineSchema schema;
        schema.globalProperties = globalProperties;
        
        for(const auto& node : nodes){

            // Create 'node' info
            gen::NodeObjInfo info;
            info.id = node->id;
            info.name = node->getName();
            info.properties = node->getProperties();
            schema.nodes.push_back(info);
        
            // Create 'connections' info
            // Loop through connections (output -> input)
            for(const auto& output : node->getOutputs() ){
                gen::NodeConnectionSchema connection;
                connection.node1Id = node->id;
                connection.node1Output = output.name;

                for(const auto& input : output.conn){
                    connection.node2Id = input.parent.id;
                    connection.node2Input = input.name;
                    schema.connections.push_back(connection);
                }
            }
        }

        // end of internal representation

        nlohmann::json j;
        nlohmann::to_json(j, schema);
        return j.dump(4);

    }

} // namespace dai

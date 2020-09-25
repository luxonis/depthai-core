#pragma once

// standard
#include <memory>

// project
#include "Node.hpp"
#include "AssetManager.hpp"

//shared
#include "depthai-shared/pb/properties/GlobalProperties.hpp"
#include "depthai-shared/pb/PipelineSchema.hpp"


namespace dai
{

    class PipelineImpl {        
        friend class Pipeline;

        AssetManager assetManager;

        GlobalProperties globalProperties;
        std::vector<std::shared_ptr<Node>> nodes;
        int64_t latestId = 0;
        int64_t getNextUniqueId();
        PipelineSchema getPipelineSchema();
        //void loadAssets(AssetManager& assetManager);
        void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage);

        AssetManager& getAssetManager();

        PipelineImpl();
        PipelineImpl(const PipelineImpl& p);
    };


    class Pipeline {

        std::shared_ptr<PipelineImpl> pimpl;

    public:
        Pipeline();
        Pipeline(const Pipeline& p);
        Pipeline(std::shared_ptr<PipelineImpl> pimpl);


        GlobalProperties getGlobalProperties() const;

        PipelineSchema getPipelineSchema();
        //void loadAssets(AssetManager& assetManager);
        void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage){
            pimpl->serialize(schema, assets, assetStorage);
        }

        std::vector<std::shared_ptr<Node>> getNodes(){
            return pimpl->nodes;
        }

        template<class N>
        std::shared_ptr<N> create(){
            static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
            auto node = std::make_shared<N>(pimpl);
            node->id = pimpl->getNextUniqueId();
            pimpl->nodes.push_back(node);
            return node;
        }

        AssetManager& getAssetManager(){
            return pimpl->getAssetManager();
        }

    };


} // namespace dai

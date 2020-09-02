#pragma once

// standard
#include <memory>

// project
#include "Node.hpp"

//shared
#include "depthai-shared/generated/GlobalProperties.hpp"


namespace dai
{

    class PipelineImpl {        
        friend class Pipeline;

        gen::GlobalProperties globalProperties;
        std::vector<std::shared_ptr<Node>> nodes;
        int64_t latestId = 0;
        int64_t getNextUniqueId();
        nlohmann::json toJson();
        std::vector<std::uint8_t> serialize();
        void loadAssets(AssetManager& assetManager);

        PipelineImpl();
        PipelineImpl(const PipelineImpl& p);
    };


    class Pipeline {
        std::shared_ptr<PipelineImpl> pimpl;

    public:
        Pipeline();
        Pipeline(const Pipeline& p);

        gen::GlobalProperties getGlobalProperties() const;

        nlohmann::json toJson();
        std::vector<std::uint8_t> serialize();
        void loadAssets(AssetManager& assetManager);

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

    };


} // namespace dai

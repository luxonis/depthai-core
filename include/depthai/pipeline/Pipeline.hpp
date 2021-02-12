#pragma once

// standard
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

// project
#include "AssetManager.hpp"
#include "Node.hpp"
#include "depthai/openvino/OpenVINO.hpp"

// shared
#include "depthai-shared/pipeline/PipelineSchema.hpp"
#include "depthai-shared/properties/GlobalProperties.hpp"

namespace dai {

class PipelineImpl {
    friend class Pipeline;
    friend class Node;

    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema() const;
    OpenVINO::Version getPipelineOpenVINOVersion() const;
    AssetManager getAllAssets() const;

    // Access to nodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const;
    std::vector<std::shared_ptr<Node>> getAllNodes();
    std::shared_ptr<const Node> getNode(Node::Id id) const;
    std::shared_ptr<Node> getNode(Node::Id id);

    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, OpenVINO::Version& version) const;
    void remove(std::shared_ptr<Node> node);

    std::vector<Node::Connection> getConnections() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Pipeline asset manager
    AssetManager assetManager;
    // Default version
    constexpr static auto DEFAULT_OPENVINO_VERSION = OpenVINO::Version::VERSION_2020_1;
    // Optionally forced version
    tl::optional<OpenVINO::Version> forceRequiredOpenVINOVersion;
    // Global pipeline properties
    GlobalProperties globalProperties;
    // Optimized for adding, searching and removing connections
    std::unordered_map<Node::Id, std::shared_ptr<Node>> nodeMap;
    using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::Connection>>;
    // Connection map, NodeId represents id of node connected TO (input)
    NodeConnectionMap nodeConnectionMap;

    // Template create function
    template <class N>
    std::shared_ptr<N> create(const std::shared_ptr<PipelineImpl>& itself) {
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        // Get unique id for this new node
        auto id = getNextUniqueId();
        // Create and store the node in the map
        auto node = std::make_shared<N>(itself, id);
        nodeMap[id] = node;
        // Return shared pointer to this node
        return node;
    }
};

class Pipeline {
    std::shared_ptr<PipelineImpl> pimpl;
    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

   public:
    Pipeline();
    explicit Pipeline(const std::shared_ptr<PipelineImpl>& pimpl);

    // Default Pipeline openvino version
    constexpr static auto DEFAULT_OPENVINO_VERSION = PipelineImpl::DEFAULT_OPENVINO_VERSION;

    GlobalProperties getGlobalProperties() const;

    PipelineSchema getPipelineSchema();
    // void loadAssets(AssetManager& assetManager);
    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, OpenVINO::Version& version) const {
        impl()->serialize(schema, assets, assetStorage, version);
    }

    template <class N>
    std::shared_ptr<N> create() {
        return impl()->create<N>(pimpl);
    }

    // Remove node capability
    void remove(std::shared_ptr<Node> node) {
        impl()->remove(node);
    }

    // getAllNodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const {
        return impl()->getAllNodes();
    }
    std::vector<std::shared_ptr<Node>> getAllNodes() {
        return impl()->getAllNodes();
    }

    // getNode
    std::shared_ptr<const Node> getNode(Node::Id id) const {
        return impl()->getNode(id);
    }
    std::shared_ptr<Node> getNode(Node::Id id) {
        return impl()->getNode(id);
    }

    std::vector<Node::Connection> getConnections() const {
        return impl()->getConnections();
    }

    using NodeConnectionMap = PipelineImpl::NodeConnectionMap;
    const NodeConnectionMap& getConnectionMap() const {
        return impl()->nodeConnectionMap;
    }

    void link(const Node::Output& out, const Node::Input& in) {
        impl()->link(out, in);
    }

    void unlink(const Node::Output& out, const Node::Input& in) {
        impl()->unlink(out, in);
    }

    AssetManager getAllAssets() const {
        return impl()->getAllAssets();
    }

    AssetManager& getAssetManager() {
        return impl()->assetManager;
    }

    const AssetManager& getAssetManager() const {
        return impl()->assetManager;
    }
};

}  // namespace dai

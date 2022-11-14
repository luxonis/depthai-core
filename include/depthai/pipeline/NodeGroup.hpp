#pragma once

// standard
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

// project
#include "AssetManager.hpp"
#include "Node.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/AtomicBool.hpp"

// shared
#include "depthai-shared/device/BoardConfig.hpp"
#include "depthai-shared/pipeline/PipelineSchema.hpp"
#include "depthai-shared/properties/GlobalProperties.hpp"

namespace dai {

class NodeGroup : public std::enable_shared_from_this<NodeGroup> {
    friend class Pipeline;
    friend class Node;

   public:
    /// Creates standalone NodeGroup
    NodeGroup() : assetManager("/node/"), {}
    /// Creates a NodeGroup with a parent NodeGroup
    NodeGroup(std::shared_ptr<NodeGroup> parentGroup) : assetManager("/node/"), parentGroup(parentGroup), {}
    /// Creates a NodeGroup with a parent Pipeline
    NodeGroup(std::shared_ptr<PipelineImpl> parentPipeline) : assetManager("/node/"), parentPipeline(parentPipeline), {}

   private:
    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();

    // Access to nodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const;
    std::vector<std::shared_ptr<Node>> getAllNodes();
    std::shared_ptr<const Node> getNode(Node::Id id) const;
    std::shared_ptr<Node> getNode(Node::Id id);
    void remove(std::shared_ptr<Node> node);
    std::vector<Node::Connection> getConnections() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    virtual Properties& getProperties();
    virtual tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion();
    /// Get a reference to internal node map
    const NodeMap& getNodeMap() const {
        return impl()->nodeMap;
    }


    // TBD to impl.
    // void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    // nlohmann::json serializeToJson() const;
    // void setCalibrationData(CalibrationHandler calibrationDataHandler);
    // CalibrationHandler getCalibrationData() const;
    // bool isHostOnly() const;
    // bool isDeviceOnly() const;

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Groups asset manager
    AssetManager assetManager;
    // Optimized for adding, searching and removing connections
    using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    NodeMap nodeMap;
    using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::Connection>>;
    // Connection map, NodeId represents id of node connected TO (input)
    NodeConnectionMap nodeConnectionMap;
    // properties holder
    copyable_unique_ptr<Properties> propertiesHolder;

    // Parents (either pipeline or group)
    std::weak_ptr<PipelineImpl> parentPipeline;
    std::weak_ptr<NodeGroup> parentGroup;

    // Template create function
    template <class N>
    std::shared_ptr<N> create() {
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value || std::is_base_of<NodeGroup, N>::value, "Specified class is not a subclass of Node or NodeGroup");
        // Get unique id for this new node/group
        auto id = getNextUniqueId();
        // Create and store the node in the map
        auto node = std::make_shared<N>(this, id);
        nodeMap[id] = node;
        // Return shared pointer to this node
        return node;
    }

    // Add a node to nodeMap
    void add(std::shared_ptr<Node> node);
    // Add a group to nodeMap
    void add(std::shared_ptr<NodeGroup> node);

    // Resource
    std::vector<uint8_t> loadResource(dai::Path uri);
    std::vector<uint8_t> loadResourceCwd(dai::Path uri, dai::Path cwd);
};


// // NodeGroup CRTP class
// template <typename Base, typename Derived, typename Props>
// class NodeGroupCRTP : public Base {
//    public:
//     using Properties = Props;
//     virtual ~NodeGroupCRTP() = default;
//     /// Underlying properties
//     Properties& properties;
//     const char* getName() const override {
//         return Derived::NAME;
//     };
//     std::unique_ptr<Node> clone() const override {
//         return std::make_unique<Derived>(static_cast<const Derived&>(*this));
//     };

//    private:
//     NodeGroupCRTP(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
//         : Base(par, nodeId, std::move(props)), properties(static_cast<Properties&>(Node::properties)) {}
//     NodeGroupCRTP(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : NodeGroupCRTP(par, nodeId, std::make_unique<Props>()) {}
//     friend Derived;
//     friend Base;
//     friend class PipelineImpl;
// };


}  // namespace dai

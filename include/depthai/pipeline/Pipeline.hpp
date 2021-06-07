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
#include "depthai/openvino/OpenVINO.hpp"

// shared
#include "depthai-shared/pipeline/PipelineSchema.hpp"
#include "depthai-shared/properties/GlobalProperties.hpp"

namespace dai {

class PipelineImpl {
    friend class Pipeline;
    friend class Node;

   public:
    PipelineImpl() = default;
    PipelineImpl(const PipelineImpl&) = default;

   private:
    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema() const;
    OpenVINO::Version getPipelineOpenVINOVersion() const;
    AssetManager getAllAssets() const;
    void setCameraTuningBlobPath(const std::string& path);

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
    void setCalibrationData(CalibrationHandler calibrationDataHandler);
    CalibrationHandler getCalibrationData() const;

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Pipeline asset manager
    AssetManager assetManager;
    // Default version
    constexpr static auto DEFAULT_OPENVINO_VERSION = OpenVINO::Version::VERSION_2021_3;
    // Optionally forced version
    tl::optional<OpenVINO::Version> forceRequiredOpenVINOVersion;
    // Global pipeline properties
    GlobalProperties globalProperties;
    // Optimized for adding, searching and removing connections
    using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    NodeMap nodeMap;
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

/**
 * @brief Represents the pipeline, set of nodes and connections between them
 */
class Pipeline {
    std::shared_ptr<PipelineImpl> pimpl;
    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

   public:
    /**
     * Constructs a new pipeline
     */
    Pipeline();
    explicit Pipeline(const std::shared_ptr<PipelineImpl>& pimpl);

    /// Clone the pipeline (Creates a copy)
    Pipeline clone() const;

    /// Default Pipeline openvino version
    constexpr static auto DEFAULT_OPENVINO_VERSION = PipelineImpl::DEFAULT_OPENVINO_VERSION;

    /**
     * @returns Global properties of current pipeline
     */
    GlobalProperties getGlobalProperties() const;

    /**
     * @returns Pipeline schema
     */
    PipelineSchema getPipelineSchema();

    // void loadAssets(AssetManager& assetManager);
    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, OpenVINO::Version& version) const {
        impl()->serialize(schema, assets, assetStorage, version);
    }

    /**
     * Adds a node to pipeline.
     *
     * Node is specified by template argument N
     */
    template <class N>
    std::shared_ptr<N> create() {
        return impl()->create<N>(pimpl);
    }

    /// Removes a node from pipeline
    void remove(std::shared_ptr<Node> node) {
        impl()->remove(node);
    }

    /// Get a vector of all nodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const {
        return impl()->getAllNodes();
    }
    /// Get a vector of all nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() {
        return impl()->getAllNodes();
    }

    /// Get node with id if it exists, nullptr otherwise
    std::shared_ptr<const Node> getNode(Node::Id id) const {
        return impl()->getNode(id);
    }
    /// Get node with id if it exists, nullptr otherwise
    std::shared_ptr<Node> getNode(Node::Id id) {
        return impl()->getNode(id);
    }

    /// Get all connections
    std::vector<Node::Connection> getConnections() const {
        return impl()->getConnections();
    }

    using NodeConnectionMap = PipelineImpl::NodeConnectionMap;
    /// Get a reference to internal connection representation
    const NodeConnectionMap& getConnectionMap() const {
        return impl()->nodeConnectionMap;
    }

    using NodeMap = PipelineImpl::NodeMap;
    /// Get a reference to internal node map
    const NodeMap& getNodeMap() const {
        return impl()->nodeMap;
    }

    /**
     * Link output to an input. Both nodes must be on the same pipeline
     *
     * Throws an error if they aren't or cannot be connected
     *
     * @param out Nodes output to connect from
     * @param in Nodes input to connect to
     */
    void link(const Node::Output& out, const Node::Input& in) {
        impl()->link(out, in);
    }

    /**
     * Unlink output from an input.
     *
     * Throws an error if link doesn't exists
     *
     * @param out Nodes output to unlink from
     * @param in Nodes input to unlink to
     */
    void unlink(const Node::Output& out, const Node::Input& in) {
        impl()->unlink(out, in);
    }

    /// Get assets on the pipeline includes nodes assets
    AssetManager getAllAssets() const {
        return impl()->getAllAssets();
    }

    /// Get pipelines AssetManager as reference
    const AssetManager& getAssetManager() const {
        return impl()->assetManager;
    }

    /// Get pipelines AssetManager as reference
    AssetManager& getAssetManager() {
        return impl()->assetManager;
    }

    /// Set a specific OpenVINO version to use with this pipeline
    void setOpenVINOVersion(OpenVINO::Version version) {
        impl()->forceRequiredOpenVINOVersion = version;
    }

    /**
     * Sets the calibration in pipeline which overrides the calibration data in eeprom
     *
     * @param calibrationDataHandler CalibrationHandler object which is loaded with calibration information.
     */
    void setCalibrationData(CalibrationHandler calibrationDataHandler) {
        impl()->setCalibrationData(calibrationDataHandler);
    }

    /**
     * gets the calibration data which is set through pipeline
     *
     * @return the calibrationHandler with calib data in the pipeline
     */
    CalibrationHandler getCalibrationData() const {
        return impl()->getCalibrationData();
    }

    /// Get required OpenVINO version to run this pipeline
    OpenVINO::Version getOpenVINOVersion() const {
        return impl()->getPipelineOpenVINOVersion();
    }

    /// Set a camera IQ (Image Quality) tuning blob, used for all cameras
    void setCameraTuningBlobPath(const std::string& path) {
        impl()->setCameraTuningBlobPath(path);
    }
};

}  // namespace dai

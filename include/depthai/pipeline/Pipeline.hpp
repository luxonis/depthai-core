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

class PipelineImpl : public std::enable_shared_from_this<PipelineImpl> {
    friend class Pipeline;
    friend class Node;

   public:
    PipelineImpl(Pipeline& pipeline) : assetManager("/pipeline/"), parent(pipeline) {}
    PipelineImpl(const PipelineImpl&) = default;
    ~PipelineImpl();

   private:
    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    tl::optional<OpenVINO::Version> getPipelineOpenVINOVersion() const;
    OpenVINO::Version getOpenVINOVersion() const;
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() const;
    bool isOpenVINOVersionCompatible(OpenVINO::Version version) const;
    Device::Config getDeviceConfig() const;
    void setCameraTuningBlobPath(const dai::Path& path);
    void setXLinkChunkSize(int sizeBytes);
    GlobalProperties getGlobalProperties() const;
    void setGlobalProperties(GlobalProperties globalProperties);
    void setSippBufferSize(int sizeBytes);
    void setSippDmaBufferSize(int sizeBytes);
    void setBoardConfig(BoardConfig board);
    BoardConfig getBoardConfig() const;

    // Access to nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() const;
    std::shared_ptr<Node> getNode(Node::Id id) const;

    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    nlohmann::json serializeToJson() const;
    void remove(std::shared_ptr<Node> node);

    std::vector<Node::Connection> getConnections() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    void setCalibrationData(CalibrationHandler calibrationDataHandler);
    bool isCalibrationDataAvailable() const;
    CalibrationHandler getCalibrationData() const;
    void setEepromData(tl::optional<EepromData> eepromData);
    tl::optional<EepromData> getEepromData() const;
    bool isHostOnly() const;
    bool isDeviceOnly() const;

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Pipeline asset manager
    AssetManager assetManager;
    // Optionally forced version
    tl::optional<OpenVINO::Version> forceRequiredOpenVINOVersion;
    // Global pipeline properties
    GlobalProperties globalProperties;
    // // Optimized for adding, searching and removing connections
    // using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    // NodeMap nodeMap;
    std::vector<std::shared_ptr<Node>> nodes;

    // TODO(themarpe) - refactor, connections are now carried by nodes instead
    using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::ConnectionInternal, Node::ConnectionInternal::Hash>>;
    // // Connection map, NodeId represents id of node connected TO (input)
    // NodeConnectionMap nodeConnectionMap;
    /// Get a reference to internal connection representation
    NodeConnectionMap getConnectionMap() const;

    // Board configuration
    BoardConfig board;

    // parent
    Pipeline& parent;

    // is pipeline running
    AtomicBool running{false};

    // was pipeline built
    AtomicBool isBuild{false};

    // TMP TMP - to be moved
    // DeviceBase for hybrid pipelines
    std::shared_ptr<Device> device;

    // Template create function
    template <class N>
    std::shared_ptr<N> create(const std::shared_ptr<PipelineImpl>& itself) {
        (void)itself;
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        // Create and store the node in the map
        auto node = N::create();
        // Add
        add(node);
        // Return shared pointer to this node
        return node;
    }

    // Add a node to nodeMap
    void add(std::shared_ptr<Node> node);

    // Run only host side, if any device nodes are present, error out
    bool isRunning() const;
    void build();
    void start();
    void wait();
    void stop();

    // Resource
    std::vector<uint8_t> loadResource(dai::Path uri);
    std::vector<uint8_t> loadResourceCwd(dai::Path uri, dai::Path cwd);
};

/**
 * @brief Represents the pipeline, set of nodes and connections between them
 */
class Pipeline {
    friend class PipelineImpl;
    std::shared_ptr<PipelineImpl> pimpl;

   public:
    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

    /**
     * Constructs a new pipeline
     */
    Pipeline();
    explicit Pipeline(std::shared_ptr<PipelineImpl> pimpl);

    /// Clone the pipeline (Creates a copy)
    Pipeline clone() const;

    /**
     * @returns Global properties of current pipeline
     */
    GlobalProperties getGlobalProperties() const {
        return impl()->getGlobalProperties();
    }

    /**
     * Sets global properties of pipeline
     */
    void setGlobalProperties(GlobalProperties globalProperties) {
        impl()->setGlobalProperties(globalProperties);
    }

    /**
     * @returns Pipeline schema
     */
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;

    // void loadAssets(AssetManager& assetManager);
    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage) const {
        impl()->serialize(schema, assets, assetStorage);
    }

    /// Returns whole pipeline represented as JSON
    nlohmann::json serializeToJson() const {
        return impl()->serializeToJson();
    }

    /**
     * Creates and adds a node to the pipeline.
     *
     * Node is specified by template argument N
     */
    template <class N>
    std::shared_ptr<N> create() {
        return impl()->create<N>(pimpl);
    }

    /**
     * Adds an existing node to the pipeline
     */
    void add(std::shared_ptr<Node> node) {
        impl()->add(node);
    }

    /// Removes a node from pipeline
    void remove(std::shared_ptr<Node> node) {
        impl()->remove(node);
    }

    /// Get a vector of all nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() const {
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
    NodeConnectionMap getConnectionMap() const {
        return impl()->getConnectionMap();
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

    /**
     * check if calib data has been set or the default will be returned
     * @return true - calib data has been set
     * @return false - calib data has not been set - default will be returned
     */
    bool isCalibrationDataAvailable() const {
        return impl()->isCalibrationDataAvailable();
    }

    /**
     * gets the eeprom data from the pipeline
     *
     * @return eepromData from the the pipeline
     */
    tl::optional<EepromData> getEepromData() const {
        return impl()->getEepromData();
    }

    /**
     * Sets the eeprom data in pipeline
     *
     * @param eepromData EepromData object that is loaded in the pipeline.
     */
    void setEepromData(tl::optional<EepromData> eepromData) {
        impl()->setEepromData(eepromData);
    }

    /// Get possible OpenVINO version to run this pipeline
    OpenVINO::Version getOpenVINOVersion() const {
        return impl()->getOpenVINOVersion();
    }

    /// Get required OpenVINO version to run this pipeline. Can be none
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() const {
        return impl()->getRequiredOpenVINOVersion();
    }

    /// Set a camera IQ (Image Quality) tuning blob, used for all cameras
    void setCameraTuningBlobPath(const dai::Path& path) {
        impl()->setCameraTuningBlobPath(path);
    }

    /**
     * Set chunk size for splitting device-sent XLink packets, in bytes. A larger value could
     * increase performance, with 0 disabling chunking. A negative value won't modify the
     * device defaults - configured per protocol, currently 64*1024 for both USB and Ethernet.
     */
    void setXLinkChunkSize(int sizeBytes) {
        impl()->setXLinkChunkSize(sizeBytes);
    }

    /**
     * SIPP (Signal Image Processing Pipeline) internal memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * By default memory is allocated in high speed CMX memory. Setting to 0 will allocate in DDR 256 kilobytes.
     * Units are bytes.
     */
    void setSippBufferSize(int sizeBytes) {
        impl()->setSippBufferSize(sizeBytes);
    }

    /**
     * SIPP (Signal Image Processing Pipeline) internal DMA memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * Memory is allocated in high speed CMX memory
     * Units are bytes.
     */
    void setSippDmaBufferSize(int sizeBytes) {
        impl()->setSippDmaBufferSize(sizeBytes);
    }

    /// Checks whether a given OpenVINO version is compatible with the pipeline
    bool isOpenVINOVersionCompatible(OpenVINO::Version version) const {
        return impl()->isOpenVINOVersionCompatible(version);
    }

    /// Sets board configuration
    void setBoardConfig(BoardConfig board) {
        impl()->setBoardConfig(board);
    }

    /// Gets board configuration
    BoardConfig getBoardConfig() const {
        return impl()->getBoardConfig();
    }

    /// Get device configuration needed for this pipeline
    Device::Config getDeviceConfig() const {
        return impl()->getDeviceConfig();
    }

    bool isRunning() const {
        return impl()->isRunning();
    }
    void build() {
        impl()->build();
    }
    void start() {
        impl()->start();
    }
    void wait() {
        impl()->wait();
    }
    void stop() {
        impl()->stop();
    }

    std::shared_ptr<Device> getDevice() {
        return impl()->device;
    }
};

}  // namespace dai

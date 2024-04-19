#pragma once

// standard
#include <memory>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

// project
#include "AssetManager.hpp"
#include "DeviceNode.hpp"
#include "Node.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/AtomicBool.hpp"

// shared
#include "depthai/device/BoardConfig.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "depthai/properties/GlobalProperties.hpp"

namespace dai {

class PipelineImpl : public std::enable_shared_from_this<PipelineImpl> {
    friend class Pipeline;
    friend class Node;

   public:
    PipelineImpl(Pipeline& pipeline, bool createImplicitDevice = false) : assetManager("/pipeline/"), parent(pipeline) {
        if(createImplicitDevice) {
            defaultDevice = std::make_shared<Device>();
        }
    }
    PipelineImpl(Pipeline& pipeline, std::shared_ptr<Device> device) : assetManager("/pipeline/"), parent(pipeline), defaultDevice{std::move(device)} {}
    PipelineImpl(const PipelineImpl&) = delete;
    PipelineImpl& operator=(const PipelineImpl&) = delete;
    PipelineImpl(PipelineImpl&&) = delete;
    PipelineImpl& operator=(PipelineImpl&&) = delete;
    ~PipelineImpl();

   private:
    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    std::optional<OpenVINO::Version> getPipelineOpenVINOVersion() const;
    OpenVINO::Version getOpenVINOVersion() const;
    std::optional<OpenVINO::Version> getRequiredOpenVINOVersion() const;
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
    std::vector<std::shared_ptr<Node>> getSourceNodes();

    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    nlohmann::json serializeToJson() const;
    void remove(std::shared_ptr<Node> node);

    std::vector<Node::Connection> getConnections() const;
    std::vector<Node::ConnectionInternal> getConnectionsInternal() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    void setCalibrationData(CalibrationHandler calibrationDataHandler);
    bool isCalibrationDataAvailable() const;
    CalibrationHandler getCalibrationData() const;
    void setEepromData(std::optional<EepromData> eepromData);
    std::optional<EepromData> getEepromData() const;
    bool isHostOnly() const;
    bool isDeviceOnly() const;

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Pipeline asset manager
    AssetManager assetManager;
    // Optionally forced version
    std::optional<OpenVINO::Version> forceRequiredOpenVINOVersion;
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

    // Add a mutex for any state change
    std::mutex stateMtx;

    // DeviceBase for hybrid pipelines
    std::shared_ptr<Device> defaultDevice;

    template <typename N, typename... Args>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> createNode(Args&&... args) {
        // N is a subclass of DeviceNode
        // return N::create();  // Specific create call for DeviceNode subclasses
        if(defaultDevice == nullptr) {
            throw std::runtime_error("Pipeline is host only, cannot create device node");
        }
        return N::create(defaultDevice, std::forward<Args>(args)...);  // Specific create call for DeviceNode subclasses
    }

    template <typename N, typename... Args>
    std::enable_if_t<!std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> createNode(Args&&... args) {
        // N is not a subclass of DeviceNode
        return N::create(std::forward<Args>(args)...);  // Generic create call
    }

    // Template create function
    template <class N, typename... Args>
    std::shared_ptr<N> create(const std::shared_ptr<PipelineImpl>& itself, Args&&... args) {
        (void)itself;
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        // Create and store the node in the map
        auto node = createNode<N>(std::forward<Args>(args)...);
        // std::shared_ptr<N> node = nullptr;
        add(node);
        // Return shared pointer to this node
        return node;
    }

    // Add a node to nodeMap
    void add(std::shared_ptr<Node> node);

    // Run only host side, if any device nodes are present, error out
    bool isRunning() const;
    bool isBuilt() const;
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
    friend class Device;

    std::shared_ptr<PipelineImpl> pimpl;

   protected:
    std::vector<std::shared_ptr<Node>> getSourceNodes() {
        return impl()->getSourceNodes();
    }

   public:
    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

    /**
     * Creates a pipeline
     * @param hostOnly If true, pipeline will run only be able to run host nodes and no device nodes can be added, otherwise pipeline implicitly creates a
     * device
     */
    explicit Pipeline(bool createImplicitDevice = false);

    /**
     * Creates a pipeline with specified device
     */
    explicit Pipeline(std::shared_ptr<Device> device);

    /**
     * Creates a pipeline with specified device
     */
    explicit Pipeline(std::shared_ptr<PipelineImpl> pimpl);

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
    template <class N, typename... Args>
    std::shared_ptr<N> create(Args&&... args) {
        return impl()->create<N>(pimpl, std::forward<Args>(args)...);
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
    std::optional<EepromData> getEepromData() const {
        return impl()->getEepromData();
    }

    /**
     * Sets the eeprom data in pipeline
     *
     * @param eepromData EepromData object that is loaded in the pipeline.
     */
    void setEepromData(std::optional<EepromData> eepromData) {
        impl()->setEepromData(eepromData);
    }

    /// Get possible OpenVINO version to run this pipeline
    OpenVINO::Version getOpenVINOVersion() const {
        return impl()->getOpenVINOVersion();
    }

    /// Get required OpenVINO version to run this pipeline. Can be none
    std::optional<OpenVINO::Version> getRequiredOpenVINOVersion() const {
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

    bool isBuilt() const {
        return impl()->isBuilt();
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

    /*
     * @note In case of a host only pipeline, this function returns a nullptr
     */
    std::shared_ptr<Device> getDefaultDevice() {
        return impl()->defaultDevice;
    }
};

}  // namespace dai

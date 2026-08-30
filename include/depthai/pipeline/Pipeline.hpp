// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

// standard
#include <chrono>
#include <memory>
#include <optional>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

// project
#include "AssetManager.hpp"
#include "DeviceNode.hpp"
#include "Node.hpp"
#include "PipelineStateApi.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/utility/AtomicBool.hpp"

// shared
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/BoardConfig.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/properties/GlobalProperties.hpp"
#include "depthai/utility/RecordReplay.hpp"

namespace dai {

namespace fs = std::filesystem;

enum class PipelineAutoCalibrationMode : int {
    OFF = 0,
    ON_START = 1,
    CONTINUOUS = 2,
};

class PipelineImpl : public std::enable_shared_from_this<PipelineImpl> {
    friend class Pipeline;
    friend class Node;
    friend class Node::Input;
    friend class DeviceBase;
    friend class utility::PipelineImplHelper;

   public:
    PipelineImpl(bool createImplicitDevice = true) : assetManager("/pipeline/") {
        if(createImplicitDevice) {
            defaultDevice = std::make_shared<Device>();
        } else {
            hostProperties = DeviceProperties();
            defaultDeviceProperties = &hostProperties.value();
        }
    }
    PipelineImpl(std::shared_ptr<Device> device) : assetManager("/pipeline/"), defaultDevice{std::move(device)} {}
    PipelineImpl(const PipelineImpl&) = delete;
    PipelineImpl& operator=(const PipelineImpl&) = delete;
    PipelineImpl(PipelineImpl&&) = delete;
    PipelineImpl& operator=(PipelineImpl&&) = delete;
    ~PipelineImpl();

   protected:
    // Record and Replay
    RecordConfig recordConfig;
    bool enableHolisticRecordReplay = false;
    std::unordered_map<std::string, std::filesystem::path> recordReplayFilenames;
    bool removeRecordReplayFiles = true;
    std::string defaultDeviceId;
    std::unordered_map<Node::Id, std::shared_ptr<Device>> bridgeHostDevices;
    // Is the pipeline building on host? Some steps should be skipped when building on device
    bool buildingOnHost = true;

    // Pipeline events
    bool enablePipelineDebugging = false;
    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;
    std::shared_ptr<MessageQueue> pipelineStateTraceOut;
    std::shared_ptr<InputQueue> pipelineStateTraceRequest;

    // Access to nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() const;
    std::shared_ptr<Node> getNode(Node::Id id) const;
    std::vector<std::shared_ptr<Node>> getSourceNodes();

   private:
    static std::string createTelemetryPipelineId();

    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE, bool includePipelineDebugging = true) const;
    PipelineSchema getDevicePipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE,
                                           bool includePipelineDebugging = true,
                                           std::optional<std::string> deviceId = std::nullopt) const;
    Device::Config getDeviceConfig() const;
    void setCameraTuningBlobPath(const fs::path& path);
    void setCameraTuningBlobPath(CameraBoardSocket socket, const fs::path& path);
    void setXLinkChunkSize(int sizeBytes);
    GlobalProperties getGlobalProperties() const;
    void setGlobalProperties(const GlobalProperties& globalProperties);
    void setDefaultDeviceProperties(const DeviceProperties& deviceProperties);
    void setDefaultDevicePropertiesRef(DeviceProperties* deviceProperties);
    std::optional<DeviceProperties> getDefaultDeviceProperties() const;
    void setSippBufferSize(int sizeBytes);
    void setSippDmaBufferSize(int sizeBytes);
    void setBoardConfig(const BoardConfig& board);
    void setAutoCalibrationMode(PipelineAutoCalibrationMode mode);
    std::pair<std::shared_ptr<dai::node::Camera>, std::shared_ptr<dai::node::Camera>> getStereoPair(const std::shared_ptr<Device>& device) const;
    bool hasDynamicCalibration() const;
    PipelineAutoCalibrationMode getAutoCalibrationMode() const;

    BoardConfig getBoardConfig() const;

    void serialize(PipelineSchema& schema,
                   Assets& assets,
                   std::vector<std::uint8_t>& assetStorage,
                   SerializationType type = DEFAULT_SERIALIZATION_TYPE,
                   std::optional<std::string> deviceId = std::nullopt) const;
    nlohmann::json serializeToJson(bool includeAssets) const;
    void remove(const std::shared_ptr<Node>& node);

    std::vector<Node::Connection> getConnections() const;
    std::vector<Node::ConnectionInternal> getConnectionsInternal() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    void setCalibrationData(const CalibrationHandler& calibrationDataHandler);
    bool isCalibrationDataAvailable() const;
    CalibrationHandler getCalibrationData() const;
    void setEepromData(const std::optional<EepromData>& eepromData);
    std::optional<EepromData> getEepromData() const;
    uint32_t getEepromId() const;
    bool isHostOnly() const;
    bool isDeviceOnly() const;
    std::vector<std::shared_ptr<Device>> getAllAssignedDevices() const;

    // Pipeline state getters
    PipelineStateApi getPipelineState();

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
    std::vector<std::pair<int64_t, int64_t>> xlinkBridges;

    // TODO(themarpe) - refactor, connections are now carried by nodes instead
    using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::ConnectionInternal, Node::ConnectionInternal::Hash>>;
    // // Connection map, NodeId represents id of node connected TO (input)
    // NodeConnectionMap nodeConnectionMap;
    /// Get a reference to internal connection representation
    NodeConnectionMap getConnectionMap() const;

    // Board configuration
    BoardConfig board;

    // Build-time automatic calibration policy for implicit AutoCalibration node creation.
    PipelineAutoCalibrationMode autoCalibrationMode = PipelineAutoCalibrationMode::ON_START;
    bool autoCalibrationModeSetByApi = false;

    // Output queues
    std::vector<std::shared_ptr<MessageQueue>> outputQueues;

    // is pipeline running
    AtomicBool running{false};
    std::string telemetryPipelineId{createTelemetryPipelineId()};
    std::optional<std::chrono::steady_clock::time_point> telemetryPipelineStartedAt;

    // was pipeline built
    AtomicBool isBuild{false};

    // Add a mutex for any state change
    std::mutex stateMtx;

    // Calibration mutex
    mutable std::mutex calibMtx;

    // DeviceBase for hybrid pipelines
    std::shared_ptr<Device> defaultDevice;
    std::optional<DeviceProperties> hostProperties;
    DeviceProperties* defaultDeviceProperties = nullptr;

    // Queue for tasks
    LockingQueue<std::function<void()>> tasks;

    // Devices that are part of this pipeline. Registered explicitly via addDevice
    // or implicitly on first use (node created with an explicit device).
    // defaultDevice is the master and is not necessarily contained here.
    std::vector<std::shared_ptr<Device>> devices;

    // Serializes concurrent per-device schema serialization (devices start in parallel)
    mutable std::mutex serializeMtx;

    // Input -> distinct source devices (nullptr entry = host), resolved from the user
    // graph at build() before bridge insertion. See Node::Input::getSourceDevice.
    std::unordered_map<Node::Input*, std::vector<std::shared_ptr<Device>>> inputSourceDevices;

    // Source device of an input: the device of the upstream node, nullptr if the
    // upstream node runs on host. Uses the map above after build, resolves live before.
    std::shared_ptr<Device> getInputSourceDevice(const Node::Input* input) const;

    // Per-device pipeline state (guarded by deviceStateMtx)
    mutable std::mutex deviceStateMtx;
    std::unordered_map<const DeviceBase*, DeviceState> deviceStates;
    std::function<void(std::shared_ptr<Device>, DeviceState)> deviceStateCallback;
    // Devices that consume other devices' streams (B side of a relay feeding a device
    // node); losing one of these for good stops the pipeline. Derived at build().
    std::unordered_set<const Device*> fatalDevices;

    // Called from a device's monitor thread on state transitions. On FAILED, idles the
    // device's XLink host nodes and stops the pipeline when the device was fatal or the
    // last one alive.
    void onDeviceStateChanged(DeviceBase* device, DeviceState state);

    DeviceState getDeviceState(const std::shared_ptr<Device>& device) const;
    void setDeviceStateCallback(std::function<void(std::shared_ptr<Device>, DeviceState)> callback);

    // Register a device with this pipeline. The first registered device is promoted
    // to master (default device) if none exists. Registering the same device twice is a no-op.
    std::shared_ptr<Device> registerDevice(std::shared_ptr<Device> device);

    // All devices that are part of this pipeline: master (default device) first,
    // then the rest in registration order.
    std::vector<std::shared_ptr<Device>> getDevices() const;

    void addTask(std::function<void()> task) {
        tasks.push(std::move(task));
    }

    void processTasks(bool waitForTasks = false, double timeoutSeconds = -1.0) {
        bool timeoutSet = timeoutSeconds >= 0.0;
        if(waitForTasks) {
            std::function<void()> task;
            bool success;
            if(timeoutSet) {
                success = tasks.tryWaitAndPop(task, std::chrono::duration<double>(timeoutSeconds));
            } else {
                success = tasks.waitAndPop(task);
            }
            if(!success) {
                return;
            }
            task();
        }
        // Regardless if we should wait or not, run all remaining tasks
        while(!tasks.empty()) {
            std::function<void()> task;
            bool success = false;
            success = tasks.tryPop(task);
            if(!success) {
                // No more tasks
                break;
            }
            task();
        }
    }

    template <typename N>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && !std::is_base_of<HostRunnable, N>::value, std::shared_ptr<N>> createNode() {
        if(defaultDevice == nullptr) {
            throw std::runtime_error("Pipeline is host only, cannot create device node");
        }
        return N::create(defaultDevice);
    }

    template <typename N, typename... Rest>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && !std::is_base_of<HostRunnable, N>::value, std::shared_ptr<N>> createNode(
        std::shared_ptr<Device> device, Rest&&... rest) {
        if(device == nullptr) {
            throw std::runtime_error("Explicit device is null");
        }
        return N::create(device, std::forward<Rest>(rest)...);
    }

    template <typename N, typename First, typename... Rest>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && !std::is_base_of<HostRunnable, N>::value
                         && !std::is_same_v<std::decay_t<First>, std::shared_ptr<Device>>,
                     std::shared_ptr<N>>
    createNode(First&& first, Rest&&... rest) {
        if(defaultDevice == nullptr) {
            throw std::runtime_error("Pipeline is host only, cannot create device node");
        }
        return N::create(defaultDevice, std::forward<First>(first), std::forward<Rest>(rest)...);
    }

    template <typename N>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && std::is_base_of<HostRunnable, N>::value, std::shared_ptr<N>> createNode() {
        if(defaultDevice == nullptr) {
            return N::create();
        } else {
            return N::create(defaultDevice);
        }
    }

    template <typename N, typename... Rest>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && std::is_base_of<HostRunnable, N>::value, std::shared_ptr<N>> createNode(
        std::shared_ptr<Device> device, Rest&&... rest) {
        if(device == nullptr) {
            throw std::runtime_error("Explicit device is null");
        }
        return N::create(device, std::forward<Rest>(rest)...);
    }

    template <typename N, typename First, typename... Rest>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value && std::is_base_of<HostRunnable, N>::value
                         && !std::is_same_v<std::decay_t<First>, std::shared_ptr<Device>>,
                     std::shared_ptr<N>>
    createNode(First&& first, Rest&&... rest) {
        if(defaultDevice == nullptr) {
            return N::create(std::forward<First>(first), std::forward<Rest>(rest)...);
        }
        return N::create(defaultDevice, std::forward<First>(first), std::forward<Rest>(rest)...);
    }

    template <typename N, typename... Args>
    std::enable_if_t<!std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> createNode(Args&&... args) {
        return N::create(std::forward<Args>(args)...);
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

    template <class N, typename... Args>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> createWithDevice(const std::shared_ptr<PipelineImpl>& itself,
                                                                                                 std::shared_ptr<Device> device,
                                                                                                 Args&&... args) {
        (void)itself;
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        if(device == nullptr) {
            throw std::runtime_error("Explicit device is null");
        }
        auto node = N::create(device, std::forward<Args>(args)...);
        add(node);
        return node;
    }

    // Add a node to nodeMap
    void add(const std::shared_ptr<Node>& node);

    /**
     * Wire subtree into this pipeline (assign ids, parent weak_ptr, default device).
     * Used when a node that is already in the pipeline gains new children (lazy subgraphs).
     */
    void adoptSubtree(std::shared_ptr<Node> root);

    // Run only host side, if any device nodes are present, error out
    bool isRunning() const;
    bool isBuilt() const;
    void build();
    void start();
    void wait();
    void stop();
    void run();

    // Reset connections and re-send the pipeline; restricted to one device when given
    void resetConnections(DeviceBase* device = nullptr);
    // Make XLink host nodes of the given device (all when null) exit quietly
    void disconnectXLinkHosts(DeviceBase* device = nullptr);

   private:
    // Resource
    std::vector<uint8_t> loadResource(const fs::path& uri);
    std::vector<uint8_t> loadResourceCwd(fs::path uri, fs::path cwd, bool moveAsset = false);
};

/**
 * @brief Represents the pipeline, set of nodes and connections between them
 */
class Pipeline {
    friend class PipelineImpl;
    friend class Device;

    std::shared_ptr<PipelineImpl> pimpl;

   public:
    using AutoCalibrationMode = PipelineAutoCalibrationMode;

    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

    std::vector<std::shared_ptr<Node>> getSourceNodes() {
        return impl()->getSourceNodes();
    }

    /**
     * Creates a pipeline
     * @param createImplicitDevice If true, creates a default device (default = true)
     */
    explicit Pipeline(bool createImplicitDevice = true);

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
     * Sets default device properties
     */
    void setDefaultDeviceProperties(DeviceProperties deviceProperties) {
        impl()->setDefaultDeviceProperties(deviceProperties);
    }

    /**
     * Sets default device properties reference. The properties should live at least as long as the pipeline.
     */
    void setDefaultDevicePropertiesRef(DeviceProperties* deviceProperties) {
        impl()->setDefaultDevicePropertiesRef(deviceProperties);
    }

    /**
     * Gets a copy of default device properties. If pipeline is in host only mode, returns host properties, otherwise returns device properties
     */
    std::optional<DeviceProperties> getDefaultDeviceProperties() const {
        return impl()->getDefaultDeviceProperties();
    }

    /**
     * @returns Pipeline schema
     */
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE, bool includePipelineDebugging = true) const;

    /**
     * @returns Device pipeline schema (without host only nodes and connections)
     */
    PipelineSchema getDevicePipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE, bool includePipelineDebugging = true) const;

    // void loadAssets(AssetManager& assetManager);
    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage) const {
        impl()->serialize(schema, assets, assetStorage);
    }

    /// Returns whole pipeline represented as JSON
    nlohmann::json serializeToJson(bool includeAssests = true) const {
        return impl()->serializeToJson(includeAssests);
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
     * Creates a device node on the specified device and adds it to the pipeline.
     * The device becomes part of the pipeline on first use. Only device nodes can be
     * created this way - creating a host node with a device is a compile-time error.
     */
    template <class N, typename... Args>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> create(std::shared_ptr<Device> device, Args&&... args) {
        return impl()->createWithDevice<N>(pimpl, std::move(device), std::forward<Args>(args)...);
    }

    /**
     * Creates a device node on the specified device and adds it to the pipeline.
     * Same as create(device, ...) - kept as an explicitly named alternative.
     */
    template <class N, typename... Args>
    std::enable_if_t<std::is_base_of<DeviceNode, N>::value, std::shared_ptr<N>> createForDevice(std::shared_ptr<Device> device, Args&&... args) {
        return impl()->createWithDevice<N>(pimpl, std::move(device), std::forward<Args>(args)...);
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
     * check if calib data is available on the device
     * @return true - calib data is available
     * @return false - calib data is not available
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

    /**
     * Gets the eeprom id from the pipeline
     *
     * @return eeprom id from the pipeline
     */
    uint32_t getEepromId() const {
        return impl()->getEepromId();
    }

    /// Set a camera IQ (Image Quality) tuning blob, used for all cameras
    void setCameraTuningBlobPath(const fs::path& path) {
        impl()->setCameraTuningBlobPath(path);
    }

    /// Set a camera IQ (Image Quality) tuning blob, used for specific board socket
    void setCameraTuningBlobPath(CameraBoardSocket socket, const fs::path& path) {
        impl()->setCameraTuningBlobPath(socket, path);
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

    /// Sets board configuration
    void setBoardConfig(BoardConfig board) {
        impl()->setBoardConfig(board);
    }

    /// Sets implicit automatic calibration policy for this pipeline.
    void setAutoCalibrationMode(AutoCalibrationMode mode) {
        impl()->setAutoCalibrationMode(mode);
    }

    /// Gets implicit automatic calibration policy for this pipeline.
    AutoCalibrationMode getAutoCalibrationMode() const {
        return impl()->getAutoCalibrationMode();
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
    void buildDevice() {
        impl()->buildingOnHost = false;
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
    void processTasks(bool waitForTasks = false, double timeoutSeconds = -1.0) {
        impl()->processTasks(waitForTasks, timeoutSeconds);
    }
    void run() {
        impl()->run();
    }
    /*
     * @note In case of a host only pipeline, this function returns a nullptr
     */
    std::shared_ptr<Device> getDefaultDevice() {
        return impl()->defaultDevice;
    }
    std::shared_ptr<const Device> getDefaultDevice() const {
        return impl()->defaultDevice;
    }

    /**
     * Add a device to the pipeline. The first device added to a pipeline created
     * without an implicit device is promoted to the default device (master), so
     * nodes created without an explicit device run on it.
     *
     * @param device Already constructed device to add
     * @returns The added device
     */
    std::shared_ptr<Device> addDevice(std::shared_ptr<Device> device);

    /**
     * Construct a device from the given device info and add it to the pipeline.
     *
     * @param deviceInfo Device info to construct the device from
     * @returns The constructed device
     */
    std::shared_ptr<Device> addDevice(const DeviceInfo& deviceInfo);

    /**
     * Construct a device from a device id, IP address or name and add it to the pipeline.
     *
     * @param idOrIpOrName Device id, IP address or name to construct the device from
     * @returns The constructed device
     */
    std::shared_ptr<Device> addDevice(const std::string& idOrIpOrName);

    /**
     * Get all devices that are part of this pipeline - the default device (master)
     * first, then the rest in registration order. Devices become part of the pipeline
     * explicitly via addDevice or implicitly on first use by a node.
     */
    std::vector<std::shared_ptr<Device>> getDevices() const;

    /**
     * Get the pipeline-level state of a device: RUNNING, DISCONNECTED, RECONNECTING
     * or FAILED. Losing a device does not stop the pipeline (its streams go idle)
     * unless it was the last device alive or a device consuming other devices' streams.
     */
    DeviceState getDeviceState(const std::shared_ptr<Device>& device) const {
        return impl()->getDeviceState(device);
    }

    /**
     * Set a callback invoked on every device state transition. The callback is invoked
     * from the affected device's monitor thread; do not block in it and do not call
     * pipeline stop/start from it directly.
     */
    void setDeviceStateCallback(std::function<void(std::shared_ptr<Device>, DeviceState)> callback) {
        impl()->setDeviceStateCallback(std::move(callback));
    }

    std::string getTelemetryPipelineId() const {
        return impl()->telemetryPipelineId;
    }

    void addTask(std::function<void()> task) {
        impl()->addTask(std::move(task));
    }

    /// Record and Replay
    void enableHolisticRecord(const RecordConfig& config);
    void enableHolisticReplay(const std::string& pathToRecording);
    bool isHolisticRecordEnabled() const;
    bool isHolisticReplayEnabled() const;

    /// Pipeline debugging
    void enablePipelineDebugging(bool enable = true);
    bool isPipelineDebuggingEnabled() const;

    // Access to pipeline state queues
    std::shared_ptr<MessageQueue> getPipelineStateOut() const;
    std::shared_ptr<InputQueue> getPipelineStateRequest() const;

    // Pipeline state getters
    PipelineStateApi getPipelineState();
};

}  // namespace dai

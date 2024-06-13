#include "depthai/pipeline/Pipeline.hpp"

#include <cstring>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/host/XLinkInHost.hpp"
#include "depthai/pipeline/node/host/XLinkOutHost.hpp"
#include "depthai/utility/HolisticRecordReplay.hpp"
#include "depthai/utility/Initialization.hpp"
#include "pipeline/datatype/ImgFrame.hpp"
#include "utility/Compression.hpp"
#include "utility/Environment.hpp"
#include "utility/Platform.hpp"
#include "utility/spdlog-fmt.hpp"

// shared
#include "depthai/pipeline/NodeConnectionSchema.hpp"

// std
#include <cassert>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_set>

// libraries
#include "spdlog/fmt/fmt.h"

// Specialization of std::hash for NodeConnectionSchema
namespace std {
template <>
struct hash<::dai::NodeConnectionSchema> {
    size_t operator()(const ::dai::NodeConnectionSchema& obj) const {
        size_t seed = 0;
        std::hash<std::int64_t> hId;
        std::hash<std::string> hStr;
        seed ^= hId(obj.node1Id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hId(obj.node2Id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.node1OutputGroup) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.node1Output) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.node2InputGroup) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.node2Input) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

}  // namespace std

namespace dai {

Node::Id PipelineImpl::getNextUniqueId() {
    return latestId++;
}

Pipeline::Pipeline(bool createImplicitDevice) : pimpl(std::make_shared<PipelineImpl>(*this, createImplicitDevice)) {}

Pipeline::Pipeline(std::shared_ptr<Device> device) : pimpl(std::make_shared<PipelineImpl>(*this, device)) {}

Pipeline::Pipeline(std::shared_ptr<PipelineImpl> pimpl) : pimpl(std::move(pimpl)) {}

PipelineSchema Pipeline::getPipelineSchema(SerializationType type) const {
    return pimpl->getPipelineSchema(type);
}

GlobalProperties PipelineImpl::getGlobalProperties() const {
    return globalProperties;
}

void PipelineImpl::setGlobalProperties(GlobalProperties globalProperties) {
    this->globalProperties = globalProperties;
}

std::shared_ptr<Node> PipelineImpl::getNode(Node::Id id) const {
    // Search all nodes
    for(const auto& node : nodes) {
        auto n = node->getNode(id);
        if(n != nullptr) {
            return n;
        }
    }
    return nullptr;
}

std::vector<std::shared_ptr<Node>> PipelineImpl::getAllNodes() const {
    std::vector<std::shared_ptr<Node>> allNodes;
    for(auto& node : nodes) {
        // Insert the node in question
        allNodes.push_back(node);
        // And its subnodes
        auto n = node->getAllNodes();
        allNodes.insert(allNodes.end(), n.begin(), n.end());
    }
    return allNodes;
}

std::vector<std::shared_ptr<Node>> PipelineImpl::getSourceNodes() {
    std::vector<std::shared_ptr<Node>> sourceNodes;
    for(auto& node : nodes) {
        if(node->isSourceNode()) {
            sourceNodes.push_back(node);
        }
    }
    return sourceNodes;
}

void PipelineImpl::serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type) const {
    // Set schema
    schema = getPipelineSchema(type);

    // Serialize all asset managers into asset storage
    assetStorage.clear();
    AssetsMutable mutableAssets;
    // Pipeline assets
    assetManager.serialize(mutableAssets, assetStorage, "/pipeline/");
    // Node assets
    for(auto& node : getAllNodes()) {
        node->getAssetManager().serialize(mutableAssets, assetStorage, fmt::format("/node/{}/", node->id));
    }

    assets = mutableAssets;
}

nlohmann::json PipelineImpl::serializeToJson() const {
    PipelineSchema schema;
    Assets assets;
    std::vector<uint8_t> assetStorage;
    serialize(schema, assets, assetStorage, SerializationType::JSON);

    nlohmann::json j;
    j["pipeline"] = schema;
    for(auto& node : j["pipeline"]["nodes"]) {
        node[1]["properties"] = nlohmann::json::parse(node[1]["properties"].get<std::vector<uint8_t>>());
    }

    j["assets"] = assets;
    j["assetStorage"] = assetStorage;
    return j;
}

PipelineImpl::NodeConnectionMap PipelineImpl::getConnectionMap() const {
    NodeConnectionMap map;

    for(const auto& node : nodes) {
        auto nodeConnMap = node->getConnectionMap();
        for(auto& kv : nodeConnMap) {
            auto& n = kv.first;
            map[n->id] = kv.second;
        }
    }

    return map;
}

std::vector<Node::ConnectionInternal> PipelineImpl::getConnectionsInternal() const {
    std::vector<Node::ConnectionInternal> conns;
    auto nodeConnectionMap = getConnectionMap();
    for(const auto& kv : nodeConnectionMap) {
        const auto& connections = kv.second;
        for(const auto& conn : connections) {
            conns.push_back(conn);
        }
    }
    return conns;
}

std::vector<Node::Connection> PipelineImpl::getConnections() const {
    auto connectionsInternal = getConnectionsInternal();
    std::vector<Node::Connection> conns;
    for(const auto& conn : connectionsInternal) {
        conns.emplace_back(conn);
    }
    return conns;
}

PipelineSchema PipelineImpl::getPipelineSchema(SerializationType type) const {
    PipelineSchema schema;
    schema.globalProperties = globalProperties;
    int latestIoId = 0;
    // Loop over all nodes, and add them to schema
    for(const auto& node : getAllNodes()) {
        // const auto& node = kv.second;
        if(std::string(node->getName()) == std::string("NodeGroup")) {
            continue;
        }
        // Check if its a host node or device node
        if(node->runOnHost()) {
            // host node, no need to serialize to a schema
            // TBD any additional changes
        } else {
            // Create 'node' info
            NodeObjInfo info;
            info.id = node->id;
            info.name = node->getName();
            info.alias = node->getAlias();
            auto parentNode = node->parentNode.lock();
            if(parentNode) {
                info.parentId = parentNode->id;
            } else {
                info.parentId = -1;
            }
            const auto& deviceNode = std::dynamic_pointer_cast<DeviceNode>(node);
            if(!deviceNode) {
                throw std::invalid_argument(fmt::format("Node '{}' should subclass DeviceNode or have hostNode == true", info.name));
            }
            deviceNode->getProperties().serialize(info.properties, type);

            // Create Io information
            auto inputs = node->getInputs();
            auto outputs = node->getOutputs();

            info.ioInfo.reserve(inputs.size() + outputs.size());

            // Add inputs
            for(const auto& input : inputs) {
                NodeIoInfo io;
                io.id = latestIoId;
                latestIoId++;
                io.blocking = input.getBlocking();
                io.queueSize = input.getMaxSize();
                io.name = input.getName();
                io.group = input.getGroup();
                auto ioKey = std::make_tuple(io.group, io.name);

                io.waitForMessage = input.getWaitForMessage();
                switch(input.getType()) {
                    case Node::Input::Type::MReceiver:
                        io.type = NodeIoInfo::Type::MReceiver;
                        break;
                    case Node::Input::Type::SReceiver:
                        io.type = NodeIoInfo::Type::SReceiver;
                        break;
                }

                if(info.ioInfo.count(ioKey) > 0) {
                    if(io.group == "") {
                        throw std::invalid_argument(fmt::format("'{}.{}' redefined. Inputs and outputs must have unique names", info.name, io.name));
                    } else {
                        throw std::invalid_argument(
                            fmt::format("'{}.{}[\"{}\"]' redefined. Inputs and outputs must have unique names", info.name, io.group, io.name));
                    }
                }
                info.ioInfo[ioKey] = io;
            }

            // Add outputs
            for(const auto& output : outputs) {
                NodeIoInfo io;
                io.id = latestIoId;
                latestIoId++;
                io.blocking = false;
                io.name = output.getName();
                io.group = output.getGroup();
                auto ioKey = std::make_tuple(io.group, io.name);

                switch(output.getType()) {
                    case Node::Output::Type::MSender:
                        io.type = NodeIoInfo::Type::MSender;
                        break;
                    case Node::Output::Type::SSender:
                        io.type = NodeIoInfo::Type::SSender;
                        break;
                }

                if(info.ioInfo.count(ioKey) > 0) {
                    if(io.group == "") {
                        throw std::invalid_argument(fmt::format("'{}.{}' redefined. Inputs and outputs must have unique names", info.name, io.name));
                    } else {
                        throw std::invalid_argument(
                            fmt::format("'{}.{}[\"{}\"]' redefined. Inputs and outputs must have unique names", info.name, io.group, io.name));
                    }
                }
                info.ioInfo[ioKey] = io;
            }

            // At the end, add the constructed node information to the schema
            schema.nodes[info.id] = info;
        }
    }

    // Create 'connections' info
    // Loop through connections (output -> input) and add them to schema

    // std::unordered_map<NodeConnectionSchema, bool> hostDeviceXLinkBridge;
    // std::unordered_map<NodeConnectionSchema, bool> deviceHostXLinkBridge;

    // auto streamName = [](std::int64_t id, std::string group, std::string name) -> std::string {
    //     if(group == "") {
    //         return fmt::format("__x_{}_{}", id, name);
    //     } else {
    //         return fmt::format("__x_{}_{}[\"{}\"]", id, group, name);
    //     }
    // };
    // Node::Id xLinkBridgeId = latestId;

    for(const auto& conn : getConnectionsInternal()) {
        NodeConnectionSchema c;
        auto outNode = conn.outputNode.lock();
        auto inNode = conn.inputNode.lock();
        c.node1Id = outNode->id;
        c.node1Output = conn.outputName;
        c.node1OutputGroup = conn.outputGroup;
        c.node2Id = inNode->id;
        c.node2Input = conn.inputName;
        c.node2InputGroup = conn.inputGroup;

        bool outputHost = outNode->runOnHost();
        bool inputHost = inNode->runOnHost();

        if(outputHost && inputHost) {
            // skip - connection between host nodes doesn't have to be represented to the device
            continue;
        }

        if(outputHost && !inputHost) {
            throw std::invalid_argument(
                fmt::format("Connection from host node '{}' to device node '{}' is not allowed during serialization.", outNode->getName(), inNode->getName()));
        }

        if(!outputHost && inputHost) {
            throw std::invalid_argument(
                fmt::format("Connection from device node '{}' to host node '{}' is not allowed during serialization.", outNode->getName(), inNode->getName()));
        }

        // add the connection to the schema
        schema.connections.push_back(c);
    }

    return schema;
}

bool PipelineImpl::isOpenVINOVersionCompatible(OpenVINO::Version version) const {
    auto ver = getPipelineOpenVINOVersion();
    if(ver) {
        return OpenVINO::areVersionsBlobCompatible(version, *ver);
    } else {
        return true;
    }
}

/// Get possible OpenVINO version to run this pipeline
OpenVINO::Version PipelineImpl::getOpenVINOVersion() const {
    return getPipelineOpenVINOVersion().value_or(OpenVINO::DEFAULT_VERSION);
}

/// Get required OpenVINO version to run this pipeline. Can be none
std::optional<OpenVINO::Version> PipelineImpl::getRequiredOpenVINOVersion() const {
    return getPipelineOpenVINOVersion();
}

std::optional<OpenVINO::Version> PipelineImpl::getPipelineOpenVINOVersion() const {
    // Loop over nodes, and get the required information
    std::optional<OpenVINO::Version> version;
    std::string lastNodeNameWithRequiredVersion = "";
    Node::Id lastNodeIdWithRequiredVersion = -1;

    for(const auto& node : nodes) {
        // Check the required openvino version
        auto requiredVersion = node->getRequiredOpenVINOVersion();
        if(requiredVersion) {
            if(forceRequiredOpenVINOVersion) {
                // Check that forced openvino version is compatible with this nodes required version
                if(!OpenVINO::areVersionsBlobCompatible(*requiredVersion, *forceRequiredOpenVINOVersion)) {
                    std::string err = fmt::format("Pipeline - '{}' node with id: {}, isn't compatible with forced OpenVINO version", node->getName(), node->id);
                    throw std::logic_error(err.c_str());
                }
            } else {
                // Keep track of required openvino versions, and make sure that they are all compatible
                if(!version) {
                    version = *requiredVersion;
                    lastNodeIdWithRequiredVersion = node->id;
                    lastNodeNameWithRequiredVersion = node->getName();
                } else {
                    // if some node already has an required version, then compare if they are compatible
                    if(!OpenVINO::areVersionsBlobCompatible(*version, *requiredVersion)) {
                        // if not compatible, then throw an error
                        std::string err = fmt::format("Pipeline - OpenVINO version required by '{}' node (id: {}), isn't compatible with '{}' node (id: {})",
                                                      lastNodeNameWithRequiredVersion,
                                                      lastNodeIdWithRequiredVersion,
                                                      node->getName(),
                                                      node->id);
                        throw std::logic_error(err.c_str());
                    }
                }
            }
        }
    }

    // After iterating over, return appropriate version
    if(forceRequiredOpenVINOVersion) {
        // Return forced version
        return forceRequiredOpenVINOVersion;
    } else if(version) {
        // Return detected version
        return version;
    } else {
        // Return null
        return std::nullopt;
    }
}

Device::Config PipelineImpl::getDeviceConfig() const {
    Device::Config config;
    config.version = getPipelineOpenVINOVersion().value_or(OpenVINO::VERSION_UNIVERSAL);
    config.board = board;
    return config;
}

void PipelineImpl::setCameraTuningBlobPath(const dai::Path& path) {
    std::string assetKey = "camTuning";

    auto asset = assetManager.set(assetKey, path);

    globalProperties.cameraTuningBlobUri = asset->getRelativeUri();
    globalProperties.cameraTuningBlobSize = static_cast<uint32_t>(asset->data.size());
}

void PipelineImpl::setXLinkChunkSize(int sizeBytes) {
    globalProperties.xlinkChunkSize = sizeBytes;
}

void PipelineImpl::setSippBufferSize(int sizeBytes) {
    globalProperties.sippBufferSize = sizeBytes;
}

void PipelineImpl::setSippDmaBufferSize(int sizeBytes) {
    globalProperties.sippDmaBufferSize = sizeBytes;
}

void PipelineImpl::setBoardConfig(BoardConfig boardCfg) {
    board = boardCfg;
}

BoardConfig PipelineImpl::getBoardConfig() const {
    return board;
}

// Remove node capability
void PipelineImpl::remove(std::shared_ptr<Node> toRemove) {
    if(toRemove->parent.lock() == nullptr) {
        throw std::invalid_argument("Cannot remove a node that is not a part of any pipeline");
    }

    if(toRemove->parent.lock() != parent.pimpl) {
        throw std::invalid_argument("Cannot remove a node that is not a part of this pipeline");
    }

    // First remove the node from the pipeline directly
    auto it = std::remove(nodes.begin(), nodes.end(), toRemove);
    nodes.erase(it, nodes.end());

    // Then also remove it from all connections & subnodes
    for(auto& node : nodes) {
        // The function below handles removing all the connections and removes the node if the node is not directly attached to a pipline
        node->remove(toRemove);
    }
}

bool PipelineImpl::isSamePipeline(const Node::Output& out, const Node::Input& in) {
    // Check whether Output 'out' and Input 'in' are on the same pipeline.
    // By checking whether their parent nodes are on same pipeline
    auto outputPipeline = out.getParent().parent.lock();
    if(outputPipeline != nullptr) {
        return (outputPipeline == in.getParent().parent.lock());
    }
    return false;
}

bool PipelineImpl::canConnect(const Node::Output& out, const Node::Input& in) {
    // First check if on same pipeline
    if(!isSamePipeline(out, in)) {
        return false;
    }

    // Check that IoType match up
    if(out.getType() == Node::Output::Type::MSender && in.getType() == Node::Input::Type::MReceiver) return false;
    if(out.getType() == Node::Output::Type::SSender && in.getType() == Node::Input::Type::SReceiver) return false;

    // Check that datatypes match up
    for(const auto& outHierarchy : out.getPossibleDatatypes()) {
        for(const auto& inHierarchy : in.possibleDatatypes) {
            // Check if datatypes match for current datatype
            if(outHierarchy.datatype == inHierarchy.datatype) return true;

            // If output can produce descendants
            if(outHierarchy.descendants && isDatatypeSubclassOf(outHierarchy.datatype, inHierarchy.datatype)) return true;

            // If input allows descendants
            if(inHierarchy.descendants && isDatatypeSubclassOf(inHierarchy.datatype, outHierarchy.datatype)) return true;
        }
    }

    // If datatypes don't match up, return false
    return false;
}

void PipelineImpl::setCalibrationData(CalibrationHandler calibrationDataHandler) {
    /* if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    } */
    globalProperties.calibData = calibrationDataHandler.getEepromData();
}

bool PipelineImpl::isCalibrationDataAvailable() const {
    return globalProperties.calibData.has_value();
}

CalibrationHandler PipelineImpl::getCalibrationData() const {
    if(globalProperties.calibData) {
        return CalibrationHandler(globalProperties.calibData.value());
    } else {
        return CalibrationHandler();
    }
}

void PipelineImpl::setEepromData(std::optional<EepromData> eepromData) {
    globalProperties.calibData = eepromData;
}

std::optional<EepromData> PipelineImpl::getEepromData() const {
    return globalProperties.calibData;
}

bool PipelineImpl::isHostOnly() const {
    bool hostOnly = true;
    for(const auto& node : nodes) {
        if(!node->runOnHost()) {
            hostOnly = false;
            break;
        }
    }
    return hostOnly;
}

bool PipelineImpl::isDeviceOnly() const {
    bool deviceOnly = true;
    for(const auto& node : nodes) {
        if(node->runOnHost()) {
            deviceOnly = false;
            break;
        }
    }
    return deviceOnly;
}

void PipelineImpl::add(std::shared_ptr<Node> node) {
    if(node == nullptr) {
        throw std::invalid_argument(fmt::format("Given node pointer is null"));
    }

    // First check if node has already been added
    auto localNodes = getAllNodes();
    for(auto& n : localNodes) {
        if(node.get() == n.get()) {
            throw std::invalid_argument(fmt::format("Node with id '{}' has already been added to the pipeline", node->id));
        }
    }

    // Go through and modify nodes and its children
    // that they are now part of this pipeline
    std::weak_ptr<PipelineImpl> curParent;
    std::queue<std::shared_ptr<Node>> search;
    search.push(node);
    while(!search.empty()) {
        auto curNode = search.front();
        search.pop();

        // Assign an ID to the node
        if(curNode->id == -1) {
            curNode->id = getNextUniqueId();
        }

        if(curNode->parent.lock() == nullptr) {
            curNode->parent = parent.pimpl;
        } else if(curNode->parent.lock() != parent.pimpl) {
            throw std::invalid_argument("Cannot add a node that is already part of another pipeline");
        }

        for(auto& n : curNode->nodeMap) {
            search.push(n);
        }
    }

    // Add to the map (node holds its children itself)
    nodes.push_back(node);
}

bool PipelineImpl::isRunning() const {
    return running;
}

bool PipelineImpl::isBuilt() const {
    return isBuild;
}

void PipelineImpl::build() {
    // TODO(themarpe) - add mutex and set running up ahead
    if(isBuild) return;
    isBuild = true;

    if(defaultDevice) {
        std::string recordPath = utility::getEnv("DEPTHAI_RECORD");
        std::string replayPath = utility::getEnv("DEPTHAI_REPLAY");

        if(defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
           || defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X) {
            try {
#ifdef DEPTHAI_MERGED_TARGET
                if(enableHolisticRecordReplay) {
                    switch(recordConfig.state) {
                        case RecordConfig::RecordReplayState::RECORD:
                            recordPath = recordConfig.outputDir;
                            replayPath = "";
                            break;
                        case RecordConfig::RecordReplayState::REPLAY:
                            recordPath = "";
                            replayPath = recordConfig.outputDir;
                            break;
                        case RecordConfig::RecordReplayState::NONE:
                            enableHolisticRecordReplay = false;
                            break;
                    }
                }

                defaultDeviceMxId = defaultDevice->getMxId();

                if(!recordPath.empty() && !replayPath.empty()) {
                    spdlog::warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
                } else if(!recordPath.empty()) {
                    if(enableHolisticRecordReplay || utility::checkRecordConfig(recordPath, recordConfig)) {
                        if(platform::checkWritePermissions(recordPath)) {
                            if(utility::setupHolisticRecord(parent, defaultDeviceMxId, recordConfig, recordReplayFilenames)) {
                                recordConfig.state = RecordConfig::RecordReplayState::RECORD;
                                spdlog::info("Record enabled.");
                            } else {
                                spdlog::warn("Could not set up holistic record. Record and replay disabled.");
                            }
                        } else {
                            spdlog::warn("DEPTHAI_RECORD path does not have write permissions. Record disabled.");
                        }
                    } else {
                        spdlog::warn("Could not successfully parse DEPTHAI_RECORD. Record disabled.");
                    }
                } else if(!replayPath.empty()) {
                    if(platform::checkPathExists(replayPath)) {
                        if(platform::checkWritePermissions(replayPath)) {
                            if(utility::setupHolisticReplay(parent, replayPath, defaultDeviceMxId, recordConfig, recordReplayFilenames)) {
                                recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
                                spdlog::info("Replay enabled.");
                            } else {
                                spdlog::warn("Could not set up holistic replay. Record and replay disabled.");
                            }
                        } else {
                            spdlog::warn("DEPTHAI_REPLAY path does not have write permissions. Replay disabled.");
                        }
                    } else {
                        spdlog::warn("DEPTHAI_REPLAY path does not exist or is invalid. Replay disabled.");
                    }
                }
#else
                recordConfig.state = RecordConfig::RecordReplayState::NONE;
                if(!recordPath.empty() || !replayPath.empty()) {
                    spdlog::warn("Merged target is required to use holistic record/replay.");
                }
#endif
            } catch(std::runtime_error& e) {
                spdlog::warn("Could not set up record / replay: {}", e.what());
            }
        } else if(enableHolisticRecordReplay || !recordPath.empty() || !replayPath.empty()) {
            throw std::runtime_error("Holistic record/replay is only supported on RVC2 devices for now.");
        }
    }

    // Go through the build stages sequentially
    for(const auto& node : getAllNodes()) {
        node->buildStage1();
    }

    for(const auto& node : getAllNodes()) {
        node->buildStage2();
    }

    for(const auto& node : getAllNodes()) {
        node->buildStage3();
    }

    // Go through all the connections and handle any
    // Host -> Device connections
    // Device -> Host connections
    // Device -> Device where the devices are not the same

    // Pseudo code
    // for each connection
    // if host -> device
    //   create XlinkIn node
    //   create xlinkOutHost node
    //   connect them
    // if device -> host
    //   create XlinkOut node
    //   create XlinkInHost node
    //   connect them
    // if device -> device
    //   if devices are not the same
    //     create XlinkOut node
    //     create XlinkInHost node
    //     create XlinkOutHost node
    //     create XlinkIn node
    //     connect them

    // Create a map of already visited nodes to only create one xlink bridge
    struct XLinkOutBridge {
        std::shared_ptr<node::XLinkOut> xLinkOut;
        std::shared_ptr<node::XLinkInHost> xLinkInHost;
    };

    struct XLinkInBridge {
        std::shared_ptr<node::XLinkOutHost> xLinkOutHost;
        std::shared_ptr<node::XLinkIn> xLinkIn;
    };

    std::unordered_map<dai::Node::Output*, XLinkOutBridge> bridgesOut;
    std::unordered_map<dai::Node::Input*, XLinkInBridge> bridgesIn;
    std::unordered_set<std::string> uniqueStreamNames;

    for(auto& connection : getConnectionsInternal()) {
        auto inNode = connection.inputNode.lock();
        auto outNode = connection.outputNode.lock();
        if(!inNode || !outNode) {
            throw std::runtime_error(fmt::format(
                "Input node in connection {}-{}_{}-{} is null", connection.inputName, connection.inputGroup, connection.outputName, connection.outputGroup));
        }
        if(!outNode->runOnHost() && inNode->runOnHost()) {
            // Check if the bridge already exists
            if(bridgesOut.count(connection.out) == 0) {  // If the bridge does not already exist, create one
                // // Create a new bridge
                bridgesOut[connection.out] = XLinkOutBridge{
                    create<node::XLinkOut>(shared_from_this()),
                    create<node::XLinkInHost>(shared_from_this()),
                };
                auto& xLinkBridge = bridgesOut[connection.out];
                auto streamName = fmt::format("__x_{}_{}", outNode->id, connection.outputName);

                // Check if the stream name is unique
                if(uniqueStreamNames.count(streamName) > 0) {
                    throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                }
                uniqueStreamNames.insert(streamName);
                xLinkBridge.xLinkOut->setStreamName(streamName);
                xLinkBridge.xLinkInHost->setStreamName(streamName);
                xLinkBridge.xLinkInHost->setConnection(defaultDevice->getConnection());
                connection.out->link(xLinkBridge.xLinkOut->input);
            }
            auto xLinkBridge = bridgesOut[connection.out];
            connection.out->unlink(*connection.in);  // Unlink the connection
            xLinkBridge.xLinkInHost->out.link(*connection.in);
        } else if(!inNode->runOnHost() && outNode->runOnHost()) {
            // Check if the bridge already exists
            if(bridgesIn.count(connection.in) == 0) {  // If the bridge does not already exist, create one
                // // Create a new bridge
                bridgesIn[connection.in] = XLinkInBridge{
                    create<node::XLinkOutHost>(shared_from_this()),
                    create<node::XLinkIn>(shared_from_this()),
                };
                auto& xLinkBridge = bridgesIn[connection.in];
                auto streamName = fmt::format("__x_{}_{}", inNode->id, connection.inputName);

                // Check if the stream name is unique
                if(uniqueStreamNames.count(streamName) > 0) {
                    throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                }
                uniqueStreamNames.insert(streamName);
                xLinkBridge.xLinkOutHost->setStreamName(streamName);
                xLinkBridge.xLinkIn->setStreamName(streamName);
                xLinkBridge.xLinkOutHost->setConnection(defaultDevice->getConnection());
                xLinkBridge.xLinkIn->out.link(*connection.in);
            }
            auto xLinkBridge = bridgesIn[connection.in];
            connection.out->unlink(*connection.in);  // Unlink the original connection
            connection.out->link(xLinkBridge.xLinkOutHost->in);
        }
    }

    // Create a vector of all nodes in the pipeline
    std::vector<std::shared_ptr<Node>> allNodes = getAllNodes();
    for(auto node : allNodes) {
        if(node->runOnHost()) {
            // Nothing special to do for host nodes
            continue;
        }
        for(auto* output : node->getOutputRefs()) {
            for(auto& queueConnection : output->getQueueConnections()) {
                // For every queue connection, if it's connected to a device node, create a bridge, if it doesn't exist
                if(bridgesOut.count(queueConnection.output) == 0) {
                    // // Create a new bridge
                    bridgesOut[queueConnection.output] = XLinkOutBridge{
                        create<node::XLinkOut>(shared_from_this()),
                        create<node::XLinkInHost>(shared_from_this()),
                    };
                    auto& xLinkBridge = bridgesOut[queueConnection.output];
                    auto streamName = fmt::format("__x_{}_{}", node->id, output->getName());

                    // Check if the stream name is unique
                    if(uniqueStreamNames.count(streamName) > 0) {
                        throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                    }
                    uniqueStreamNames.insert(streamName);
                    xLinkBridge.xLinkOut->setStreamName(streamName);
                    xLinkBridge.xLinkInHost->setStreamName(streamName);
                    xLinkBridge.xLinkInHost->setConnection(defaultDevice->getConnection());
                    queueConnection.output->link(xLinkBridge.xLinkOut->input);
                }
                auto xLinkBridge = bridgesOut[queueConnection.output];
                queueConnection.output->unlink(queueConnection.queue);  // Unlink the original connection
                xLinkBridge.xLinkInHost->out.link(queueConnection.queue);
            }
        }
    }

    // Build
    if(!isHostOnly()) {
        // TODO(Morato) - handle multiple devices correctly, start pipeline on all of them
        defaultDevice->startPipeline(Pipeline(shared_from_this()));
    }
}

void PipelineImpl::start() {
    std::lock_guard<std::mutex> lock(stateMtx);
    // TODO(themarpe) - add mutex and set running up ahead

    // TODO(Morato) - add back in when more nodes are tested
    // for(const auto& node : getAllNodes()) {
    //     if (node->needsBuild()) {
    //         throw std::runtime_error(fmt::format("Node '{}' was not built", node->getName()));
    //     }
    // }

    // Implicitly build (if not already)
    build();

    // Indicate that pipeline is running
    running = true;

    // Starts pipeline, go through all nodes and start them
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->start();
        }
    }
}

void PipelineImpl::wait() {
    // Waits for all nodes to finish the execution
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->wait();
        }
    }
}

void PipelineImpl::stop() {
    std::lock_guard<std::mutex> lock(stateMtx);
    // Stops the pipeline execution
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->stop();
        }
    }

    // Close the task queue
    tasks.destruct();
    // TODO(Morato) - handle multiple devices correctly, stop pipeline on all of them
    // Close the devices
    if(!isHostOnly()) {
        defaultDevice->close();
    }

    // Indicate that pipeline is not runnin
    running = false;
}

PipelineImpl::~PipelineImpl() {
    stop();
    wait();

    if(recordConfig.state == RecordConfig::RecordReplayState::RECORD) {
        std::vector<std::string> filenames = {recordReplayFilenames["record_config"]};
        std::vector<std::string> outFiles = {"record_config.json"};
        filenames.reserve(recordReplayFilenames.size() * 2 + 1);
        outFiles.reserve(recordReplayFilenames.size() * 2 + 1);
        for(auto& rstr : recordReplayFilenames) {
            if(rstr.first != "record_config") {
                std::string nodeName = rstr.first;
                std::string filePath = rstr.second;
                filenames.push_back(filePath + ".mp4");
                filenames.push_back(filePath + ".mcap");
                outFiles.push_back(nodeName + ".mp4");
                outFiles.push_back(nodeName + ".mcap");
            }
        }
        spdlog::info("Record: Creating tar file with {} files", filenames.size());
        utility::tarFiles(platform::joinPaths(recordConfig.outputDir, "recording.tar.gz"), filenames, outFiles);
        std::remove(platform::joinPaths(recordConfig.outputDir, "record_config.json").c_str());
    }

    if(recordConfig.state != RecordConfig::RecordReplayState::NONE) {
        spdlog::info("Record and Replay: Removing temporary files");
        for(auto& kv : recordReplayFilenames) {
            if(kv.first != "record_config") {
                std::remove((kv.second + ".mp4").c_str());
                std::remove((kv.second + ".mcap").c_str());
            } else
                std::remove(kv.second.c_str());
        }
    }
}

void PipelineImpl::run() {
    start();
    while(isRunning()) {
        processTasks(true);
    }
    wait();
}

std::vector<uint8_t> PipelineImpl::loadResource(dai::Path uri) {
    return loadResourceCwd(uri, "/pipeline");
}

static dai::Path getAbsUri(dai::Path& uri, dai::Path& cwd) {
    int colonLocation = uri.string().find(":");
    std::string resourceType = uri.string().substr(0, colonLocation + 1);
    dai::Path absAssetUri;
    if(uri.string()[colonLocation + 1] == '/') {  // Absolute path
        absAssetUri = uri;
    } else {  // Relative path
        absAssetUri = dai::Path{resourceType + cwd.string() + uri.string().substr(colonLocation + 1)};
    }
    return absAssetUri;
}

std::vector<uint8_t> PipelineImpl::loadResourceCwd(dai::Path uri, dai::Path cwd) {
    struct ProtocolHandler {
        const char* protocol = nullptr;
        std::function<std::vector<uint8_t>(PipelineImpl&, const dai::Path&)> handle = nullptr;
    };

    const std::vector<ProtocolHandler> protocolHandlers = {
        {"asset",
         [](PipelineImpl& p, const dai::Path& uri) -> std::vector<uint8_t> {
             // First check the pipeline asset manager
             auto asset = p.assetManager.get(uri.u8string());
             if(asset != nullptr) {
                 return asset->data;
             }
             // If asset not found in the pipeline asset manager, check all nodes
             else {
                 for(auto& node : p.nodes) {
                     auto& assetManager = node->getAssetManager();
                     auto asset = assetManager.get(uri.u8string());
                     if(asset != nullptr) {
                         return asset->data;
                     }
                 }
             }
             // Asset not found anywhere
             throw std::invalid_argument(fmt::format("No asset with key ({}) found", uri));
         }} /*, TODO (read from filesystem 'file://' or default scheme, ...) */
    };

    for(const auto& handler : protocolHandlers) {
        std::string protocolPrefix = std::string(handler.protocol) + ":";

        if(uri.u8string().find(protocolPrefix) == 0) {
            // // protocol matches, resolve URI and call handler
            // std::filesystem::path path(uri.substr(protocolPrefix.size()));
            // // Create full path, and normalize
            // // If path is relative, otherwise path will be taken as absolute
            // auto fullPath = (cwd / path).lexically_normal();
            // // Call handler and return
            // return handler.handle(this, fullPath.string());

            // TODO(themarpe) - use above approach instead
            dai::Path path;
            if(protocolPrefix == "asset:") {
                auto absUri = getAbsUri(uri, cwd);
                path = static_cast<dai::Path>(absUri.u8string().substr(protocolPrefix.size()));
            } else {
                path = static_cast<dai::Path>(uri.u8string().substr(protocolPrefix.size()));
            }
            return handler.handle(*this, path.u8string());
        }
    }

    // If no handler executed, then return nullptr
    throw std::invalid_argument(fmt::format("No handler specified for following ({}) URI", uri));
}

// Record and Replay
void Pipeline::enableHolisticRecord(const RecordConfig& config) {
    if(this->isRunning()) {
        throw std::runtime_error("Cannot enable record while pipeline is running");
    }
    if(!platform::checkPathExists(config.outputDir, true)) {
        throw std::runtime_error("Record output directory does not exist or is invalid");
    }
    impl()->recordConfig = config;
    impl()->recordConfig.state = RecordConfig::RecordReplayState::RECORD;
    impl()->enableHolisticRecordReplay = true;
}

void Pipeline::enableHolisticReplay(const std::string& pathToRecording) {
    if(this->isRunning()) {
        throw std::runtime_error("Cannot enable replay while pipeline is running");
    }
    if(!platform::checkPathExists(pathToRecording, false)) {
        throw std::runtime_error("Replay file does not exist or is invalid");
    }
    impl()->recordConfig.outputDir = pathToRecording;
    impl()->recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
    impl()->enableHolisticRecordReplay = true;
}

}  // namespace dai

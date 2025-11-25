#include "depthai/pipeline/Pipeline.hpp"

#include <cstring>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkInHost.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "depthai/pipeline/node/internal/XLinkOutHost.hpp"
#include "depthai/utility/Initialization.hpp"
#include "pipeline/datatype/ImgFrame.hpp"
#include "pipeline/node/DetectionNetwork.hpp"
#include "pipeline/node/internal/PipelineStateMerge.hpp"
#include "utility/Compression.hpp"
#include "utility/Environment.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/HolisticRecordReplay.hpp"
#include "utility/Logging.hpp"
#include "utility/Platform.hpp"
#include "utility/RecordReplayImpl.hpp"
#include "utility/Serialization.hpp"
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

namespace fs = std::filesystem;

Node::Id PipelineImpl::getNextUniqueId() {
    return latestId++;
}

Pipeline::Pipeline(bool createImplicitDevice) : pimpl(std::make_shared<PipelineImpl>(*this, createImplicitDevice)) {}

Pipeline::Pipeline(std::shared_ptr<Device> device) : pimpl(std::make_shared<PipelineImpl>(*this, device)) {}

Pipeline::Pipeline(std::shared_ptr<PipelineImpl> pimpl) : pimpl(std::move(pimpl)) {}

PipelineSchema Pipeline::getPipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    return pimpl->getPipelineSchema(type, includePipelineDebugging);
}

PipelineSchema Pipeline::getDevicePipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    return pimpl->getDevicePipelineSchema(type, includePipelineDebugging);
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
    schema = getDevicePipelineSchema(type);

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

nlohmann::json PipelineImpl::serializeToJson(bool includeAssets) const {
    PipelineSchema schema;
    Assets assets;
    std::vector<uint8_t> assetStorage;
    serialize(schema, assets, assetStorage, SerializationType::JSON);

    nlohmann::json j;
    j["pipeline"] = schema;
    for(auto& node : j["pipeline"]["nodes"]) {
        node[1]["properties"] = nlohmann::json::parse(node[1]["properties"].get<std::vector<uint8_t>>());
    }
    if(includeAssets) {
        j["assets"] = assets;
        j["assetStorage"] = assetStorage;
    }
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

PipelineSchema PipelineImpl::getPipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    PipelineSchema schema;
    schema.globalProperties = globalProperties;
    schema.bridges = xlinkBridges;
    int latestIoId = 0;

    std::vector<Node::Id> pipelineDebuggingNodeIds;
    if(!includePipelineDebugging) {
        for(const auto& node : getAllNodes()) {
            if(std::string(node->getName()) == std::string("PipelineEventAggregation") || std::string(node->getName()) == std::string("PipelineStateMerge")) {
                pipelineDebuggingNodeIds.push_back(node->id);
            }
        }
        for(const auto& conn : getConnectionsInternal()) {
            auto outNode = conn.outputNode.lock();
            auto inNode = conn.inputNode.lock();
            if(conn.outputName == "pipelineEventOutput") continue;
            if(std::string(outNode->getName()).find("XLink") != std::string::npos || std::string(inNode->getName()).find("XLink") != std::string::npos
               || std::string(outNode->getName()).find("InputQueue") != std::string::npos
               || std::string(inNode->getName()).find("OutputQueue") != std::string::npos) {
                if(std::find(pipelineDebuggingNodeIds.begin(), pipelineDebuggingNodeIds.end(), inNode->id) != pipelineDebuggingNodeIds.end()) {
                    pipelineDebuggingNodeIds.push_back(outNode->id);
                }
                if(std::find(pipelineDebuggingNodeIds.begin(), pipelineDebuggingNodeIds.end(), outNode->id) != pipelineDebuggingNodeIds.end()) {
                    pipelineDebuggingNodeIds.push_back(inNode->id);
                }
            }
        }
    }

    // Loop over all nodes, and add them to schema
    for(const auto& node : getAllNodes()) {
        // const auto& node = kv.second;
        if(std::string(node->getName()) == std::string("NodeGroup") || std::string(node->getName()) == std::string("DeviceNodeGroup")) {
            continue;
        }
        if(!includePipelineDebugging
           && std::find(pipelineDebuggingNodeIds.begin(), pipelineDebuggingNodeIds.end(), node->id) != pipelineDebuggingNodeIds.end()) {
            continue;
        }
        // Create 'node' info
        NodeObjInfo info;
        info.id = node->id;
        info.name = node->getName();
        info.alias = node->getAlias();
        info.parentId = node->parentId;
        info.deviceNode = !node->runOnHost();
        if(!node->runOnHost()) info.deviceId = defaultDeviceId;

        const auto& deviceNode = std::dynamic_pointer_cast<DeviceNode>(node);
        if(!node->runOnHost() && !deviceNode) {
            throw std::invalid_argument(fmt::format("Node '{}' should subclass DeviceNode or have hostNode == true", info.name));
        }
        if(deviceNode) {
            deviceNode->getProperties().serialize(info.properties, type);
            info.logLevel = deviceNode->getLogLevel();
        }
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
            if(!includePipelineDebugging && output.getName() == "pipelineEventOutput") continue;
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
        if(!includePipelineDebugging && conn.outputName == "pipelineEventOutput") continue;
        NodeConnectionSchema c;
        auto outNode = conn.outputNode.lock();
        auto inNode = conn.inputNode.lock();
        c.node1Id = outNode->id;
        c.node1Output = conn.outputName;
        c.node1OutputGroup = conn.outputGroup;
        c.node2Id = inNode->id;
        c.node2Input = conn.inputName;
        c.node2InputGroup = conn.inputGroup;

        if(!includePipelineDebugging
           && (std::find(pipelineDebuggingNodeIds.begin(), pipelineDebuggingNodeIds.end(), c.node1Id) != pipelineDebuggingNodeIds.end()
               || std::find(pipelineDebuggingNodeIds.begin(), pipelineDebuggingNodeIds.end(), c.node2Id) != pipelineDebuggingNodeIds.end())) {
            continue;
        }

        bool outputHost = outNode->runOnHost();
        bool inputHost = inNode->runOnHost();

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

PipelineSchema PipelineImpl::getDevicePipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    auto schema = getPipelineSchema(type, includePipelineDebugging);
    // Remove bridge info
    schema.bridges.clear();
    // Remove host nodes
    for(auto it = schema.nodes.begin(); it != schema.nodes.end();) {
        if(!it->second.deviceNode) {
            it = schema.nodes.erase(it);
        } else {
            ++it;
        }
    }
    // Remove connections between host nodes (host - device connections should not exist)
    schema.connections.erase(std::remove_if(schema.connections.begin(),
                                            schema.connections.end(),
                                            [&schema](const NodeConnectionSchema& c) {
                                                auto node1 = schema.nodes.find(c.node1Id);
                                                auto node2 = schema.nodes.find(c.node2Id);
                                                if(node1 == schema.nodes.end() && node2 == schema.nodes.end()) {
                                                    return true;
                                                } else if(node1 == schema.nodes.end() || node2 == schema.nodes.end()) {
                                                    throw std::invalid_argument("Connection from host node to device node should not exist here");
                                                }
                                                return false;
                                            }),
                             schema.connections.end());
    return schema;
}

Device::Config PipelineImpl::getDeviceConfig() const {
    Device::Config config;
    config.board = board;
    return config;
}

void PipelineImpl::setCameraTuningBlobPath(const fs::path& path) {
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
    DAI_CHECK_V(!isBuilt(), "Cannot remove node from pipeline once it is built.");
    DAI_CHECK_V(toRemove->parent.lock() != nullptr, "Cannot remove a node that is not a part of any pipeline");
    DAI_CHECK_V(toRemove->parent.lock() == parent.pimpl, "Cannot remove a node that is not a part of this pipeline");

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
    setEepromData(calibrationDataHandler.getEepromData());
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
    std::unique_lock<std::mutex> lock(calibMtx);
    globalProperties.calibData = eepromData;
    globalProperties.eepromId += 1;  // Increment eepromId to indicate that eeprom data has changed
}

std::optional<EepromData> PipelineImpl::getEepromData() const {
    return globalProperties.calibData;
}

uint32_t PipelineImpl::getEepromId() const {
    std::unique_lock<std::mutex> lock(calibMtx);
    return globalProperties.eepromId;
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

PipelineStateApi PipelineImpl::getPipelineState() {
    bool hasPipelineMergeNode = false;
    for(const auto& node : getAllNodes()) {
        if(strcmp(node->getName(), "PipelineStateMerge") == 0) {
            hasPipelineMergeNode = true;
            break;
        }
    }
    if(!hasPipelineMergeNode) {
        throw std::runtime_error("Pipeline debugging disabled. Cannot get pipeline state.");
    }
    return PipelineStateApi(pipelineStateOut, pipelineStateRequest, getAllNodes());
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

        // In case we have a device node without an assigned device (usually subnodes in non-DeviceNode nodes), use the default device
        if(std::dynamic_pointer_cast<DeviceNode>(curNode) != nullptr && std::dynamic_pointer_cast<DeviceNode>(curNode)->getDevice() == nullptr) {
            std::dynamic_pointer_cast<DeviceNode>(curNode)->setDevice(defaultDevice);
        }

        for(auto& n : curNode->nodeMap) {
            n->parentId = curNode->id;  // Set node parent id
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

    if(buildingOnHost) {
        if(defaultDevice) {
            auto recordPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_RECORD", ""));
            auto replayPath = std::filesystem::path(utility::getEnvAs<std::string>("DEPTHAI_REPLAY", ""));

            if(defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
               || defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X
               || defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_RVC4) {
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

                    defaultDeviceId = defaultDevice->getDeviceId();

                    if(!recordPath.empty() && !replayPath.empty()) {
                        Logging::getInstance().logger.warn("Both DEPTHAI_RECORD and DEPTHAI_REPLAY are set. Record and replay disabled.");
                    } else if(!recordPath.empty()) {
                        if(enableHolisticRecordReplay || utility::checkRecordConfig(recordPath, recordConfig)) {
                            if(platform::checkWritePermissions(recordPath)) {
                                if(utility::setupHolisticRecord(parent,
                                                                defaultDeviceId,
                                                                recordConfig,
                                                                recordReplayFilenames,
                                                                defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
                                                                    || defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X)) {
                                    recordConfig.state = RecordConfig::RecordReplayState::RECORD;
                                    Logging::getInstance().logger.info("Record enabled.");
                                } else {
                                    Logging::getInstance().logger.warn("Could not set up holistic record. Record and replay disabled.");
                                }
                            } else {
                                Logging::getInstance().logger.warn("DEPTHAI_RECORD path does not have write permissions. Record disabled.");
                            }
                        } else {
                            Logging::getInstance().logger.warn("Could not successfully parse DEPTHAI_RECORD. Record disabled.");
                        }
                    } else if(!replayPath.empty()) {
                        if(platform::checkPathExists(replayPath)) {
                            if(platform::checkWritePermissions(replayPath)) {
                                if(utility::setupHolisticReplay(parent,
                                                                replayPath,
                                                                defaultDeviceId,
                                                                recordConfig,
                                                                recordReplayFilenames,
                                                                defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_2
                                                                    || defaultDevice->getDeviceInfo().platform == XLinkPlatform_t::X_LINK_MYRIAD_X)) {
                                    recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
                                    if(platform::checkPathExists(replayPath, true)) {
                                        removeRecordReplayFiles = false;
                                    }
                                    Logging::getInstance().logger.info("Replay enabled.");
                                } else {
                                    Logging::getInstance().logger.warn("Could not set up holistic replay. Record and replay disabled.");
                                }
                            } else {
                                Logging::getInstance().logger.warn("DEPTHAI_REPLAY path does not have write permissions. Replay disabled.");
                            }
                        } else {
                            Logging::getInstance().logger.warn("DEPTHAI_REPLAY path does not exist or is invalid. Replay disabled.");
                        }
                    }
#else
                    recordConfig.state = RecordConfig::RecordReplayState::NONE;
                    if(!recordPath.empty() || !replayPath.empty()) {
                        Logging::getInstance().logger.warn("Merged target is required to use holistic record/replay.");
                    }
#endif
                } catch(std::runtime_error& e) {
                    Logging::getInstance().logger.warn("Could not set up record / replay: {}", e.what());
                }
            } else if(enableHolisticRecordReplay || !recordPath.empty() || !replayPath.empty()) {
                throw std::runtime_error("Holistic record/replay is only supported on RVC2 devices for now.");
            }
        }
    }

    // Run first build stage for all nodes
    for(const auto& node : getAllNodes()) {
        node->buildStage1();
    }

    if(buildingOnHost) setupPipelineDebugging();

    {
        auto allNodes = getAllNodes();
        if(std::find_if(allNodes.begin(), allNodes.end(), [](const std::shared_ptr<Node>& n) { return strcmp(n->getName(), "PipelineEventAggregation") == 0; })
           == allNodes.end()) {
            for(auto& node : allNodes) node->pipelineEventDispatcher->sendEvents = false;
        }
    }

    isBuild = true;

    // Go through the build stages sequentially
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
        std::shared_ptr<node::internal::XLinkOut> xLinkOut;
        std::shared_ptr<node::internal::XLinkInHost> xLinkInHost;
    };

    struct XLinkInBridge {
        std::shared_ptr<node::internal::XLinkOutHost> xLinkOutHost;
        std::shared_ptr<node::internal::XLinkIn> xLinkIn;
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
                    create<node::internal::XLinkOut>(shared_from_this()),
                    create<node::internal::XLinkInHost>(shared_from_this()),
                };
                auto& xLinkBridge = bridgesOut[connection.out];
                auto streamName = fmt::format("__x_{}_{}_{}", outNode->id, connection.outputGroup, connection.outputName);

                // Check if the stream name is unique
                if(uniqueStreamNames.count(streamName) > 0) {
                    throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                }
                uniqueStreamNames.insert(streamName);
                xLinkBridge.xLinkOut->setStreamName(streamName);
                xLinkBridge.xLinkInHost->setStreamName(streamName);
                xLinkBridge.xLinkInHost->setConnection(defaultDevice->getConnection());
                connection.out->link(xLinkBridge.xLinkOut->input);

                // Note the created bridge for serialization (for visualization)
                xlinkBridges.push_back({xLinkBridge.xLinkOut->id, xLinkBridge.xLinkInHost->id});
            }
            auto xLinkBridge = bridgesOut[connection.out];
            connection.out->unlink(*connection.in);  // Unlink the connection
            xLinkBridge.xLinkInHost->out.link(*connection.in);
        } else if(!inNode->runOnHost() && outNode->runOnHost()) {
            // Check if the bridge already exists
            if(bridgesIn.count(connection.in) == 0) {  // If the bridge does not already exist, create one
                // // Create a new bridge
                bridgesIn[connection.in] = XLinkInBridge{
                    create<node::internal::XLinkOutHost>(shared_from_this()),
                    create<node::internal::XLinkIn>(shared_from_this()),
                };
                auto& xLinkBridge = bridgesIn[connection.in];
                auto streamName = fmt::format("__x_{}_{}_{}", inNode->id, connection.inputGroup, connection.inputName);

                // Check if the stream name is unique
                if(uniqueStreamNames.count(streamName) > 0) {
                    throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                }
                uniqueStreamNames.insert(streamName);
                xLinkBridge.xLinkOutHost->setStreamName(streamName);
                xLinkBridge.xLinkIn->setStreamName(streamName);
                xLinkBridge.xLinkOutHost->setConnection(defaultDevice->getConnection());
                xLinkBridge.xLinkIn->out.link(*connection.in);
                if(defaultDevice->getPlatform() == Platform::RVC4 || defaultDevice->getPlatform() == Platform::RVC3) {
                    xLinkBridge.xLinkOutHost->allowStreamResize(true);
                } else {
                    xLinkBridge.xLinkOutHost->allowStreamResize(false);
                }

                // Note the created bridge for serialization (for visualization)
                xlinkBridges.push_back({xLinkBridge.xLinkOutHost->id, xLinkBridge.xLinkIn->id});
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
                        create<node::internal::XLinkOut>(shared_from_this()),
                        create<node::internal::XLinkInHost>(shared_from_this()),
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

    // Finish setting up pipeline debugging
    if(buildingOnHost && enablePipelineDebugging) {
        // Enable events on xlink bridges
        std::shared_ptr<node::internal::PipelineEventAggregation> pipelineEventAggHost = nullptr;
        std::shared_ptr<node::internal::PipelineEventAggregation> pipelineEventAggDevice = nullptr;
        for(const auto& node : getAllNodes()) {
            if(strcmp(node->getName(), "PipelineEventAggregation") == 0) {
                if(node->runOnHost() && !pipelineEventAggHost) {
                    pipelineEventAggHost = std::dynamic_pointer_cast<node::internal::PipelineEventAggregation>(node);
                } else if(!node->runOnHost() && !pipelineEventAggDevice) {
                    pipelineEventAggDevice = std::dynamic_pointer_cast<node::internal::PipelineEventAggregation>(node);
                }
            }
            if(pipelineEventAggHost && pipelineEventAggDevice) {
                break;
            }
        }
        if(!pipelineEventAggHost || !pipelineEventAggDevice) {
            throw std::runtime_error("PipelineEventAggregation nodes not found for pipeline debugging setup");
        }
        for(auto& bridge : bridgesOut) {
            auto& nodes = bridge.second;
            nodes.xLinkInHost->pipelineEventOutput.link(
                pipelineEventAggHost->inputs[fmt::format("{} - {}", nodes.xLinkInHost->getName(), nodes.xLinkInHost->id)]);
            nodes.xLinkOut->pipelineEventOutput.link(pipelineEventAggDevice->inputs[fmt::format("{} - {}", nodes.xLinkOut->getName(), nodes.xLinkOut->id)]);
        }
        for(auto& bridge : bridgesIn) {
            auto& nodes = bridge.second;
            nodes.xLinkIn->pipelineEventOutput.link(pipelineEventAggDevice->inputs[fmt::format("{} - {}", nodes.xLinkIn->getName(), nodes.xLinkIn->id)]);
            nodes.xLinkOutHost->pipelineEventOutput.link(
                pipelineEventAggHost->inputs[fmt::format("{} - {}", nodes.xLinkOutHost->getName(), nodes.xLinkOutHost->id)]);
        }
    }

    // Initialize event dispatchers
    for(const auto& node : getAllNodes()) {
        auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(node);
        if(threadedNode) {
            threadedNode->initPipelineEventDispatcher(threadedNode->id);
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

    Logging::getInstance().logger.debug("Full schema dump: {}", ((nlohmann::json)getPipelineSchema(SerializationType::JSON, false)).dump());

    // Indicate that pipeline is running
    running = true;

    // Starts pipeline, go through all nodes and start them
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->start();
        }
    }

    // Add pointer to the pipeline to the device
    if(defaultDevice) {
        std::shared_ptr<PipelineImpl> shared = shared_from_this();
        const auto weak = std::weak_ptr<PipelineImpl>(shared);
        defaultDevice->pipelinePtr = weak;
    }

    if(buildingOnHost && utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false)) {
        if(pipelineStateTraceOut) {
            getPipelineState().configureTraceOutput(1);
            pipelineStateTraceOut->addCallback([](const std::shared_ptr<ADatatype>& data) {
                if(data) {
                    auto state = std::dynamic_pointer_cast<const PipelineState>(data);
                    if(state) Logging::getInstance().logger.trace("Pipeline state update: {}", state->toJson().dump());
                }
            });
        }
    }
}

void PipelineImpl::resetConnections() {
    // reset connection on all nodes
    if(defaultDevice->getConnection() == nullptr) throw std::runtime_error("Connection lost");
    auto con = defaultDevice->getConnection();
    for(auto node : getAllNodes()) {
        auto tmp = std::dynamic_pointer_cast<node::internal::XLinkInHost>(node);
        if(tmp) tmp->setConnection(con);
        auto tmp2 = std::dynamic_pointer_cast<node::internal::XLinkOutHost>(node);
        if(tmp2) tmp2->setConnection(con);
    }

    // restart pipeline
    if(!isHostOnly()) {
        defaultDevice->startPipeline(Pipeline(shared_from_this()));
    }
}

void PipelineImpl::disconnectXLinkHosts() {
    // make connections throw instead of reconnecting
    for(auto node : getAllNodes()) {
        auto tmp = std::dynamic_pointer_cast<node::internal::XLinkInHost>(node);
        if(tmp) tmp->disconnect();
        auto tmp2 = std::dynamic_pointer_cast<node::internal::XLinkOutHost>(node);
        if(tmp2) tmp2->disconnect();
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
    if(!running) {
        return;
    }
    // Stops the pipeline execution
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->stop();
        }
    }

    // Close all the output queues
    for(auto& queue : outputQueues) {
        queue->close();
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
        std::vector<std::filesystem::path> filenames = {recordReplayFilenames["record_config"]};
        std::vector<std::string> outFiles = {"record_config.json"};
        filenames.reserve(recordReplayFilenames.size() * 2 + 1);
        outFiles.reserve(recordReplayFilenames.size() * 2 + 1);
        for(auto& rstr : recordReplayFilenames) {
            if(rstr.first != "record_config") {
                std::string nodeName = rstr.first.substr(2);
                std::filesystem::path filePath = rstr.second;
                filenames.push_back(std::filesystem::path(filePath).concat(".mcap"));
                outFiles.push_back(nodeName + ".mcap");
                if(rstr.first[0] == 'v') {
                    filenames.push_back(std::filesystem::path(filePath).concat(".mp4"));
                    outFiles.push_back(nodeName + ".mp4");
                }
            }
        }
        Logging::getInstance().logger.info("Record: Creating tar file with {} files", filenames.size());
        try {
            utility::tarFiles(platform::joinPaths(recordConfig.outputDir, "recording.tar"), filenames, outFiles);
        } catch(const std::exception& e) {
            Logging::getInstance().logger.error("Record: Failed to create tar file: {}", e.what());
        }
        std::filesystem::remove(platform::joinPaths(recordConfig.outputDir, "record_config.json"));
    }

    if(removeRecordReplayFiles && recordConfig.state != RecordConfig::RecordReplayState::NONE) {
        Logging::getInstance().logger.info("Record and Replay: Removing temporary files");
        for(auto& kv : recordReplayFilenames) {
            if(kv.first != "record_config") {
                std::filesystem::remove(std::filesystem::path(kv.second).concat(".mcap"));
                std::filesystem::remove(std::filesystem::path(kv.second).concat(".mp4"));
            } else {
                std::filesystem::remove(kv.second);
            }
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

std::vector<uint8_t> PipelineImpl::loadResource(fs::path uri) {
    return loadResourceCwd(uri, "/pipeline");
}

static fs::path getAbsUri(fs::path& uri, fs::path& cwd) {
    int colonLocation = uri.string().find(":");
    std::string resourceType = uri.string().substr(0, colonLocation + 1);
    fs::path absAssetUri;
    if(uri.string()[colonLocation + 1] == '/') {  // Absolute path
        absAssetUri = uri;
    } else {  // Relative path
        absAssetUri = fs::path{resourceType + cwd.string() + uri.string().substr(colonLocation + 1)};
    }
    return absAssetUri;
}

std::vector<uint8_t> PipelineImpl::loadResourceCwd(fs::path uri, fs::path cwd, bool moveAsset) {
    struct ProtocolHandler {
        const char* protocol = nullptr;
        std::function<std::vector<uint8_t>(PipelineImpl&, const fs::path&)> handle = nullptr;
    };

    const std::vector<ProtocolHandler> protocolHandlers = {
        {"asset",
         [moveAsset](PipelineImpl& p, const fs::path& uri) -> std::vector<uint8_t> {
             // First check the pipeline asset manager
             auto asset = p.assetManager.get(uri.u8string());
             if(asset != nullptr) {
                 if(moveAsset) {
                     p.assetManager.remove(uri.u8string());
                     return std::move(asset->data);
                 }
                 return asset->data;
             }
             for(auto& node : p.nodes) {
                 auto& assetManager = node->getAssetManager();
                 auto asset = assetManager.get(uri.u8string());
                 if(asset != nullptr) {
                     if(moveAsset) {
                         assetManager.remove(uri.u8string());
                         return std::move(asset->data);
                     }
                     return asset->data;
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
            fs::path path;
            if(protocolPrefix == "asset:") {
                auto absUri = getAbsUri(uri, cwd);
                path = static_cast<fs::path>(absUri.u8string().substr(protocolPrefix.size()));
            } else {
                path = static_cast<fs::path>(uri.u8string().substr(protocolPrefix.size()));
            }
            return handler.handle(*this, path);
        }
    }

    // If no handler executed, then return nullptr
    throw std::invalid_argument(fmt::format("No handler specified for following ({}) URI", uri));
}

void PipelineImpl::setupPipelineDebugging() {
    // Create pipeline event aggregator node and link
    bool envPipelineDebugging = utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false);
    enablePipelineDebugging = enablePipelineDebugging || envPipelineDebugging;
    if(enablePipelineDebugging) {
        // Check if any nodes are on host or device
        bool hasDeviceNodes = false;
        for(const auto& node : getAllNodes()) {
            if(std::string(node->getName()) == std::string("NodeGroup") || std::string(node->getName()) == std::string("DeviceNodeGroup")) continue;

            if(!node->runOnHost()) {
                hasDeviceNodes = true;
            }
        }
        std::shared_ptr<node::internal::PipelineEventAggregation> hostEventAgg = nullptr;
        std::shared_ptr<node::internal::PipelineEventAggregation> deviceEventAgg = nullptr;
        hostEventAgg = parent.create<node::internal::PipelineEventAggregation>();
        hostEventAgg->setRunOnHost(true);
        hostEventAgg->setTraceOutput(envPipelineDebugging);
        if(hasDeviceNodes) {
            deviceEventAgg = parent.create<node::internal::PipelineEventAggregation>();
            deviceEventAgg->setRunOnHost(false);
            deviceEventAgg->setTraceOutput(envPipelineDebugging);
        }
        for(auto& node : getAllNodes()) {
            if(std::string(node->getName()) == std::string("NodeGroup") || std::string(node->getName()) == std::string("DeviceNodeGroup")) continue;

            auto threadedNode = std::dynamic_pointer_cast<ThreadedNode>(node);
            if(threadedNode) {
                if(node->runOnHost() && hostEventAgg && node->id != hostEventAgg->id) {
                    threadedNode->pipelineEventOutput.link(hostEventAgg->inputs[fmt::format("{} - {}", node->getName(), node->id)]);
                } else if(!node->runOnHost() && deviceEventAgg && node->id != deviceEventAgg->id) {
                    threadedNode->pipelineEventOutput.link(deviceEventAgg->inputs[fmt::format("{} - {}", node->getName(), node->id)]);
                }
            }
        }
        auto stateMerge = parent.create<node::PipelineStateMerge>()->build(hasDeviceNodes, true);
        std::shared_ptr<node::PipelineStateMerge> traceStateMerge;
        if(envPipelineDebugging) {
            traceStateMerge = parent.create<node::PipelineStateMerge>()->build(hasDeviceNodes, true);
            traceStateMerge->setAllowConfiguration(false);
        }
        if(deviceEventAgg) {
            deviceEventAgg->out.link(stateMerge->inputDevice);
            stateMerge->outRequest.link(deviceEventAgg->request);
            if(envPipelineDebugging) {
                deviceEventAgg->outTrace.link(traceStateMerge->inputDevice);
            }
        }
        if(hostEventAgg) {
            hostEventAgg->out.link(stateMerge->inputHost);
            stateMerge->outRequest.link(hostEventAgg->request);
            if(envPipelineDebugging) {
                hostEventAgg->outTrace.link(traceStateMerge->inputHost);
            }
        }
        pipelineStateOut = stateMerge->out.createOutputQueue(1, false);
        pipelineStateRequest = stateMerge->request.createInputQueue();
        if(envPipelineDebugging) pipelineStateTraceOut = traceStateMerge->out.createOutputQueue(1, false);
    }
}

// Record and Replay
void Pipeline::enableHolisticRecord(const RecordConfig& config) {
    if(this->isRunning()) {
        throw std::runtime_error("Cannot enable record while pipeline is running");
    }
    if(impl()->enableHolisticRecordReplay && impl()->recordConfig.state == RecordConfig::RecordReplayState::REPLAY) {
        throw std::runtime_error("Cannot enable record while replay is enabled");
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
    if(impl()->enableHolisticRecordReplay && impl()->recordConfig.state == RecordConfig::RecordReplayState::RECORD) {
        throw std::runtime_error("Cannot enable replay while record is enabled");
    }
    if(!platform::checkPathExists(pathToRecording, false)) {
        throw std::runtime_error("Replay file does not exist or is invalid");
    }
    impl()->recordConfig.outputDir = pathToRecording;
    impl()->recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
    impl()->enableHolisticRecordReplay = true;
}

void Pipeline::enablePipelineDebugging(bool enable) {
    if(utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false)) {
        throw std::runtime_error(
            "You can enable pipeline debugging either through the DEPTHAI_PIPELINE_DEBUGGING environment variable or through the Pipeline API, not both");
    }
    if(this->isBuilt()) {
        throw std::runtime_error("Cannot change pipeline debugging state after pipeline is built");
    }
    impl()->enablePipelineDebugging = enable;
}

PipelineStateApi Pipeline::getPipelineState() {
    return impl()->getPipelineState();
}

}  // namespace dai

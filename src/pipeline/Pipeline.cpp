#include "depthai/pipeline/Pipeline.hpp"

#include <cstring>

#include "depthai/device/CalibrationHandler.hpp"
#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
    #include "depthai/pipeline/node/AutoCalibration.hpp"
#endif
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkInHost.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "depthai/pipeline/node/internal/XLinkOutHost.hpp"
#include "properties/GlobalProperties.hpp"
#include "utility/Compression.hpp"
#include "utility/Environment.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/HolisticRecordReplay.hpp"
#include "utility/Logging.hpp"
#include "utility/PipelineImplHelper.hpp"
#include "utility/Platform.hpp"
#include "utility/RecordReplayImpl.hpp"
#include "utility/Serialization.hpp"
#include "utility/Telemetry.hpp"
#include "utility/Uuid.hpp"
#include "utility/depthai_nodes_names.hpp"

// shared
#include "depthai/pipeline/NodeConnectionSchema.hpp"

// std
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
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

namespace {

std::shared_ptr<Device> getAssignedDevice(const std::shared_ptr<Node>& node) {
    auto deviceNode = std::dynamic_pointer_cast<DeviceNode>(node);
    if(!deviceNode) {
        return nullptr;
    }
    return deviceNode->getDevice();
}

std::string pathToUtf8String(const fs::path& path) {
#ifdef _WIN32
    const auto utf8 = path.u8string();
    return {reinterpret_cast<const char*>(utf8.data()), utf8.size()};
#else
    return path.string();
#endif
}

#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
const char* autoCalibrationModeToString(PipelineAutoCalibrationMode mode) {
    switch(mode) {
        case PipelineAutoCalibrationMode::OFF:
            return "OFF";
        case PipelineAutoCalibrationMode::ON_START:
            return "ON_START";
        case PipelineAutoCalibrationMode::CONTINUOUS:
            return "CONTINUOUS";
    }
    return "ON_START";
}

std::optional<PipelineAutoCalibrationMode> parseAutoCalibrationMode(std::string_view mode) {
    std::string modeStr(mode);
    std::transform(modeStr.begin(), modeStr.end(), modeStr.begin(), [](char c) { return static_cast<char>(std::toupper(static_cast<unsigned char>(c))); });
    if(modeStr == "OFF") {
        return PipelineAutoCalibrationMode::OFF;
    }
    if(modeStr == "ON_START") {
        return PipelineAutoCalibrationMode::ON_START;
    }
    if(modeStr == "CONTINUOUS") {
        return PipelineAutoCalibrationMode::CONTINUOUS;
    }
    return std::nullopt;
}
#endif

PipelineSchema anonymizeCustomNodesForTelemetry(PipelineSchema schema) {
    for(auto& node : schema.nodes) {
        const auto isDepthaiNodesNode =
            std::find(utility::depthaiNodesNames.begin(), utility::depthaiNodesNames.end(), node.second.name) != utility::depthaiNodesNames.end();
        if(!node.second.builtInNode && !isDepthaiNodesNode) {
            node.second.name = "CUSTOM";
        }
    }
    return schema;
}

bool redactTelemetryNodeProperties(const std::string& nodeName) {
    return nodeName == "CUSTOM" || nodeName == "DetectionParser" || nodeName == "SegmentationParser";
}

nlohmann::json makeTelemetrySchemaJson(PipelineSchema schema) {
    auto telemetrySchema = nlohmann::json(anonymizeCustomNodesForTelemetry(std::move(schema)));
    for(auto& node : telemetrySchema["nodes"]) {
        auto& nodeInfo = node.at(1);
        const auto nodeName = nodeInfo.value("name", std::string{});
        if(redactTelemetryNodeProperties(nodeName)) {
            nodeInfo["properties"] = "REDACTED";
        }
    }
    return telemetrySchema;
}

nlohmann::json makeTelemetryNodePropertiesJson(const nlohmann::json& properties) {
    if(properties.is_string() && properties.get<std::string>() == "REDACTED") {
        return nlohmann::json{{"redacted", true}};
    }
    if(properties.is_object()) {
        return properties;
    }
    if(!properties.is_array() || properties.empty()) {
        return nlohmann::json::object();
    }

    try {
        const auto propertyBytes = properties.get<std::vector<std::uint8_t>>();
        if(propertyBytes.empty()) {
            return nlohmann::json::object();
        }
        auto parsedProperties = nlohmann::json::parse(propertyBytes.begin(), propertyBytes.end());
        if(parsedProperties.is_object()) {
            return parsedProperties;
        }
    } catch(const std::exception& ex) {
        logger::debug("Failed to parse node telemetry properties as JSON: {}", ex.what());
    }
    return nlohmann::json::object();
}

void emitTelemetryNodeCreatedEvents(const Pipeline& pipeline, const nlohmann::json& telemetrySchema) {
    for(const auto& node : telemetrySchema.at("nodes")) {
        const auto& nodeInfo = node.at(1);
        dai::utility::Telemetry::getInstance().event(pipeline,
                                                     "depthai_node_created",
                                                     nlohmann::json{
                                                         {"name", nodeInfo.value("name", std::string{})},
                                                         {"properties", makeTelemetryNodePropertiesJson(nodeInfo.value("properties", nlohmann::json{}))},
                                                     });
    }
}

#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
bool hasDifferentDistortion(const CalibrationHandler& lhs, const CalibrationHandler& rhs, CameraBoardSocket socket) {
    if(!lhs.hasCameraCalibration(socket) || !rhs.hasCameraCalibration(socket)) {
        if(!lhs.hasCameraCalibration(socket) && !rhs.hasCameraCalibration(socket)) {
            return false;
        }
        return true;
    }

    if(lhs.getDistortionModel(socket) != rhs.getDistortionModel(socket)) {
        return true;
    }

    const auto lhsDist = lhs.getDistortionCoefficients(socket);
    const auto rhsDist = rhs.getDistortionCoefficients(socket);

    if(lhsDist.size() != rhsDist.size()) {
        return true;
    }

    constexpr float EPS = 1e-6f;
    for(size_t i = 0; i < lhsDist.size(); ++i) {
        if(std::fabs(lhsDist[i] - rhsDist[i]) > EPS) {
            return true;
        }
    }

    return false;
}
#endif

}  // namespace

namespace fs = std::filesystem;

std::mutex pipelineBuildMutex;

std::string PipelineImpl::createTelemetryPipelineId() {
    return utility::generateUuidV7();
}

Node::Id PipelineImpl::getNextUniqueId() {
    return latestId++;
}

Pipeline::Pipeline(bool createImplicitDevice) : pimpl(std::make_shared<PipelineImpl>(createImplicitDevice)) {}

Pipeline::Pipeline(std::shared_ptr<Device> device) : pimpl(std::make_shared<PipelineImpl>(device)) {}

Pipeline::Pipeline(std::shared_ptr<PipelineImpl> pimpl) : pimpl(std::move(pimpl)) {}

std::shared_ptr<Device> Pipeline::addDevice(std::shared_ptr<Device> device) {
    return impl()->registerDevice(std::move(device));
}

std::shared_ptr<Device> Pipeline::addDevice(const DeviceInfo& deviceInfo) {
    return impl()->registerDevice(std::make_shared<Device>(deviceInfo));
}

std::shared_ptr<Device> Pipeline::addDevice(const std::string& idOrIpOrName) {
    return impl()->registerDevice(std::make_shared<Device>(idOrIpOrName));
}

std::vector<std::shared_ptr<Device>> Pipeline::getDevices() const {
    return impl()->getDevices();
}

void Pipeline::setCameraTuningBlobPath(const std::shared_ptr<Device>& device, const fs::path& path) {
    impl()->setCameraTuningBlobPath(device, path);
}

void Pipeline::setCameraTuningBlobPath(const std::shared_ptr<Device>& device, CameraBoardSocket socket, const fs::path& path) {
    impl()->setCameraTuningBlobPath(device, socket, path);
}

void Pipeline::setXLinkChunkSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    impl()->setXLinkChunkSize(device, sizeBytes);
}

void Pipeline::setSippBufferSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    impl()->setSippBufferSize(device, sizeBytes);
}

void Pipeline::setSippDmaBufferSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    impl()->setSippDmaBufferSize(device, sizeBytes);
}

void Pipeline::setBoardConfig(const std::shared_ptr<Device>& device, const BoardConfig& board) {
    impl()->setBoardConfig(device, board);
}

BoardConfig Pipeline::getBoardConfig(const std::shared_ptr<Device>& device) const {
    return impl()->getBoardConfig(device);
}

Device::Config Pipeline::getDeviceConfig(const std::shared_ptr<Device>& device) const {
    return impl()->getDeviceConfig(device);
}

void Pipeline::setCalibrationData(const std::shared_ptr<Device>& device, const CalibrationHandler& calibrationDataHandler) {
    impl()->setCalibrationData(device, calibrationDataHandler);
}

bool Pipeline::isCalibrationDataAvailable(const std::shared_ptr<Device>& device) const {
    return impl()->isCalibrationDataAvailable(device);
}

CalibrationHandler Pipeline::getCalibrationData(const std::shared_ptr<Device>& device) const {
    return impl()->getCalibrationData(device);
}

void Pipeline::setEepromData(const std::shared_ptr<Device>& device, const std::optional<EepromData>& eepromData) {
    impl()->setEepromData(device, eepromData);
}

std::optional<EepromData> Pipeline::getEepromData(const std::shared_ptr<Device>& device) const {
    return impl()->getEepromData(device);
}

uint32_t Pipeline::getEepromId(const std::shared_ptr<Device>& device) const {
    return impl()->getEepromId(device);
}

void Pipeline::setDeviceProperties(const std::shared_ptr<Device>& device, const DeviceProperties& deviceProperties) {
    impl()->setDeviceProperties(device, deviceProperties);
}

DeviceProperties Pipeline::getDeviceProperties(const std::shared_ptr<Device>& device) const {
    return impl()->getDeviceProperties(device);
}

PipelineSchema Pipeline::getPipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    return pimpl->getPipelineSchema(type, includePipelineDebugging);
}

PipelineSchema Pipeline::getDevicePipelineSchema(SerializationType type, bool includePipelineDebugging) const {
    return pimpl->getDevicePipelineSchema(type, includePipelineDebugging);
}

GlobalProperties PipelineImpl::getGlobalProperties() const {
    return globalProperties;
}

void PipelineImpl::setGlobalProperties(const GlobalProperties& globalProperties) {
    this->globalProperties = globalProperties;
}

void PipelineImpl::setDefaultDeviceProperties(const DeviceProperties& deviceProperties) {
    if(defaultDevice) {
        defaultDevice->setProperties(deviceProperties);
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->setFrom(deviceProperties);
    }
}

void PipelineImpl::setDefaultDevicePropertiesRef(DeviceProperties* deviceProperties) {
    if(deviceProperties == nullptr) {
        throw std::runtime_error("deviceProperties pointer cannot be null.");
    }
    defaultDeviceProperties = deviceProperties;
}

std::optional<DeviceProperties> PipelineImpl::getDefaultDeviceProperties() const {
    if(defaultDevice) {
        return defaultDevice->getProperties();
    } else if(defaultDeviceProperties != nullptr) {
        return *defaultDeviceProperties;
    }
    return std::nullopt;
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

void PipelineImpl::serialize(
    PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type, std::optional<std::string> deviceId) const {
    // Devices are started in parallel and node property serialization is not thread safe
    std::lock_guard<std::mutex> lock(serializeMtx);

    // Set schema
    schema = getDevicePipelineSchema(type, true, deviceId);

    // Only ship a device the assets of nodes that run on it; without a device filter include all
    auto includeNodeAssets = [&deviceId](const std::shared_ptr<Node>& node) {
        if(!deviceId.has_value()) return true;
        if(node->runOnHost()) return false;
        auto deviceNode = std::dynamic_pointer_cast<DeviceNode>(node);
        if(deviceNode == nullptr || deviceNode->getDevice() == nullptr) return false;
        return deviceNode->getDevice()->getDeviceInfo().getDeviceId() == *deviceId;
    };

    // Serialize all asset managers into asset storage
    assetStorage.clear();
    std::size_t storageSize = assetManager.getSerializedSize();
    for(auto& node : getAllNodes()) {
        if(!includeNodeAssets(node)) continue;
        storageSize = node->getAssetManager().getSerializedSize(storageSize);
    }
    assetStorage.reserve(storageSize);
    AssetsMutable mutableAssets;
    // Pipeline assets
    assetManager.serialize(mutableAssets, assetStorage, "/pipeline/");
    // Node assets
    for(auto& node : getAllNodes()) {
        if(!includeNodeAssets(node)) continue;
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
        info.builtInNode = node->isBuiltInNode();
        if(!node->runOnHost()) info.deviceId = defaultDeviceId;

        const auto& deviceNode = std::dynamic_pointer_cast<DeviceNode>(node);
        if(!node->runOnHost() && !deviceNode) {
            throw std::invalid_argument(fmt::format("Node '{}' should subclass DeviceNode or have hostNode == true", info.name));
        }
        if(deviceNode) {
            auto device = deviceNode->getDevice();
            if(!node->runOnHost() && device == nullptr) {
                throw std::invalid_argument(fmt::format("Device node '{}' has no assigned device", info.name));
            }
            if(device != nullptr) {
                // Stable host-side identifier, no RPC. Must stay consistent with the
                // deviceId used to filter per-device schemas in DeviceBase::startPipelineImpl.
                info.deviceId = device->getDeviceInfo().getDeviceId();
            }
            deviceNode->getProperties().serialize(info.properties, type);
            if(std::string(deviceNode->getName()) == "DeviceNodeGroup") {
                info.logLevel = LogLevel::OFF;
            } else {
                info.logLevel = deviceNode->getLogLevel();
            }
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

PipelineSchema PipelineImpl::getDevicePipelineSchema(SerializationType type, bool includePipelineDebugging, std::optional<std::string> deviceId) const {
    auto schema = getPipelineSchema(type, includePipelineDebugging);
    // Remove bridge info
    schema.bridges.clear();
    // Remove host and group nodes
    for(auto it = schema.nodes.begin(); it != schema.nodes.end();) {
        const bool wrongDevice = deviceId.has_value() && it->second.deviceId != *deviceId;
        if(!it->second.deviceNode || it->second.name == "NodeGroup" || it->second.name == "DeviceNodeGroup" || wrongDevice) {
            it = schema.nodes.erase(it);
        } else {
            ++it;
        }
    }
    // Remove connections touching nodes outside of this device-local schema.
    schema.connections.erase(std::remove_if(schema.connections.begin(),
                                            schema.connections.end(),
                                            [&schema](const NodeConnectionSchema& c) {
                                                auto node1 = schema.nodes.find(c.node1Id);
                                                auto node2 = schema.nodes.find(c.node2Id);
                                                return node1 == schema.nodes.end() || node2 == schema.nodes.end();
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

    if(defaultDevice) {
        defaultDevice->setCameraTuningBlob(asset->getRelativeUri(), static_cast<uint32_t>(asset->getSize()));
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->cameraTuningBlobUri = asset->getRelativeUri();
        defaultDeviceProperties->cameraTuningBlobSize = static_cast<uint32_t>(asset->getSize());
    }
}

void PipelineImpl::setCameraTuningBlobPath(CameraBoardSocket socket, const fs::path& path) {
    std::string assetKey = "camTuning";
    assetKey += "_" + std::to_string(static_cast<int>(socket));

    auto asset = assetManager.set(assetKey, path);

    if(defaultDevice) {
        defaultDevice->setCameraSocketTuningBlob(socket, asset->getRelativeUri(), static_cast<uint32_t>(asset->getSize()));
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->cameraSocketTuningBlobUri[socket] = asset->getRelativeUri();
        defaultDeviceProperties->cameraSocketTuningBlobSize[socket] = static_cast<uint32_t>(asset->getSize());
    }
}

void PipelineImpl::setXLinkChunkSize(int sizeBytes) {
    if(defaultDevice) {
        defaultDevice->setXLinkChunkSize(sizeBytes);
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->xlinkChunkSize = sizeBytes;
    }
}

void PipelineImpl::setSippBufferSize(int sizeBytes) {
    if(defaultDevice) {
        defaultDevice->setSippBufferSize(sizeBytes);
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->sippBufferSize = sizeBytes;
    }
}

void PipelineImpl::setSippDmaBufferSize(int sizeBytes) {
    if(defaultDevice) {
        defaultDevice->setSippDmaBufferSize(sizeBytes);
    } else if(defaultDeviceProperties != nullptr) {
        defaultDeviceProperties->sippDmaBufferSize = sizeBytes;
    }
}

void PipelineImpl::setBoardConfig(const BoardConfig& boardCfg) {
    board = boardCfg;
}

void PipelineImpl::setAutoCalibrationMode(PipelineAutoCalibrationMode mode) {
    DAI_CHECK_V(!isBuilt(), "Cannot change auto calibration mode once the pipeline is built.");
    autoCalibrationMode = mode;
    autoCalibrationModeSetByApi = true;
}

PipelineAutoCalibrationMode PipelineImpl::getAutoCalibrationMode() const {
    return autoCalibrationMode;
}

BoardConfig PipelineImpl::getBoardConfig() const {
    return board;
}

// Remove node capability
void PipelineImpl::remove(const std::shared_ptr<Node>& toRemove) {
    DAI_CHECK_V(!isBuilt(), "Cannot remove node from pipeline once it is built.");
    DAI_CHECK_V(toRemove->parent.lock() != nullptr, "Cannot remove a node that is not a part of any pipeline");
    DAI_CHECK_V(toRemove->parent.lock() == shared_from_this(), "Cannot remove a node that is not a part of this pipeline");

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

void PipelineImpl::setCalibrationData(const CalibrationHandler& calibrationDataHandler) {
    setEepromData(calibrationDataHandler.getEepromData());
}

bool PipelineImpl::isCalibrationDataAvailable() const {
    if(defaultDevice) {
        return defaultDevice->isCalibrationAvailable();
    }
    if(defaultDeviceProperties != nullptr) {
        std::lock_guard<std::mutex> lock(calibMtx);
        return defaultDeviceProperties->calibData.has_value();
    }
    return false;
}

CalibrationHandler PipelineImpl::getCalibrationData() const {
    if(defaultDevice) {
        return defaultDevice->getCalibration();
    }
    if(defaultDeviceProperties != nullptr && defaultDeviceProperties->calibData.has_value()) {
        std::lock_guard<std::mutex> lock(calibMtx);
        if(defaultDeviceProperties->calibData.has_value()) {
            return CalibrationHandler(defaultDeviceProperties->calibData.value());
        }
    }
    throw std::runtime_error("No default device properties set in pipeline");
}

void PipelineImpl::setEepromData(const std::optional<EepromData>& eepromData) {
    if(defaultDevice) {
        defaultDevice->setCalibration(eepromData);
    } else if(defaultDeviceProperties != nullptr) {
        std::lock_guard<std::mutex> lock(calibMtx);
        defaultDeviceProperties->calibData = eepromData;
        defaultDeviceProperties->eepromId += 1;  // Increment eepromId to indicate that eeprom data has changed
    }
}

std::optional<EepromData> PipelineImpl::getEepromData() const {
    if(defaultDevice) {
        if(auto calibration = defaultDevice->tryGetCalibration()) {
            return calibration->getEepromData();
        }
        return std::nullopt;
    }
    if(defaultDeviceProperties != nullptr) {
        std::unique_lock<std::mutex> lock(calibMtx);
        return defaultDeviceProperties->calibData;
    }
    return std::nullopt;
}

uint32_t PipelineImpl::getEepromId() const {
    if(defaultDevice) {
        return defaultDevice->getProperties().eepromId;
    }
    if(defaultDeviceProperties != nullptr) {
        std::unique_lock<std::mutex> lock(calibMtx);
        return defaultDeviceProperties->eepromId;
    }
    return 0;
}

std::shared_ptr<Device> PipelineImpl::resolvePipelineDevice(const std::shared_ptr<Device>& device) const {
    if(device == nullptr) {
        throw std::invalid_argument("Device is null");
    }
    if(device == defaultDevice || std::find(devices.begin(), devices.end(), device) != devices.end()) {
        return device;
    }
    throw std::invalid_argument("Device is not part of this pipeline - add it with addDevice or use it with a node first");
}

Device::Config PipelineImpl::getDeviceConfig(const std::shared_ptr<Device>& device) const {
    resolvePipelineDevice(device);
    Device::Config config;
    config.board = getBoardConfig(device);
    return config;
}

void PipelineImpl::setCameraTuningBlobPath(const std::shared_ptr<Device>& device, const fs::path& path) {
    resolvePipelineDevice(device);
    // Per-device asset key so different devices can carry different tuning blobs
    std::string assetKey = "camTuning_" + device->getDeviceInfo().getDeviceId();
    auto asset = assetManager.set(assetKey, path);
    device->setCameraTuningBlob(asset->getRelativeUri(), static_cast<uint32_t>(asset->getSize()));
}

void PipelineImpl::setCameraTuningBlobPath(const std::shared_ptr<Device>& device, CameraBoardSocket socket, const fs::path& path) {
    resolvePipelineDevice(device);
    std::string assetKey = "camTuning_" + device->getDeviceInfo().getDeviceId() + "_" + std::to_string(static_cast<int>(socket));
    auto asset = assetManager.set(assetKey, path);
    device->setCameraSocketTuningBlob(socket, asset->getRelativeUri(), static_cast<uint32_t>(asset->getSize()));
}

void PipelineImpl::setXLinkChunkSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    resolvePipelineDevice(device)->setXLinkChunkSize(sizeBytes);
}

void PipelineImpl::setSippBufferSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    resolvePipelineDevice(device)->setSippBufferSize(sizeBytes);
}

void PipelineImpl::setSippDmaBufferSize(const std::shared_ptr<Device>& device, int sizeBytes) {
    resolvePipelineDevice(device)->setSippDmaBufferSize(sizeBytes);
}

void PipelineImpl::setBoardConfig(const std::shared_ptr<Device>& device, const BoardConfig& boardCfg) {
    resolvePipelineDevice(device);
    if(device == defaultDevice) {
        board = boardCfg;
    } else {
        deviceBoardConfigs[device.get()] = boardCfg;
    }
}

BoardConfig PipelineImpl::getBoardConfig(const std::shared_ptr<Device>& device) const {
    resolvePipelineDevice(device);
    if(device == defaultDevice) {
        return board;
    }
    auto it = deviceBoardConfigs.find(device.get());
    if(it != deviceBoardConfigs.end()) {
        return it->second;
    }
    return BoardConfig{};
}

void PipelineImpl::setCalibrationData(const std::shared_ptr<Device>& device, const CalibrationHandler& calibrationDataHandler) {
    setEepromData(device, calibrationDataHandler.getEepromData());
}

bool PipelineImpl::isCalibrationDataAvailable(const std::shared_ptr<Device>& device) const {
    return resolvePipelineDevice(device)->isCalibrationAvailable();
}

CalibrationHandler PipelineImpl::getCalibrationData(const std::shared_ptr<Device>& device) const {
    return resolvePipelineDevice(device)->getCalibration();
}

void PipelineImpl::setEepromData(const std::shared_ptr<Device>& device, const std::optional<EepromData>& eepromData) {
    resolvePipelineDevice(device)->setCalibration(eepromData);
}

std::optional<EepromData> PipelineImpl::getEepromData(const std::shared_ptr<Device>& device) const {
    if(auto calibration = resolvePipelineDevice(device)->tryGetCalibration()) {
        return calibration->getEepromData();
    }
    return std::nullopt;
}

uint32_t PipelineImpl::getEepromId(const std::shared_ptr<Device>& device) const {
    return resolvePipelineDevice(device)->getProperties().eepromId;
}

void PipelineImpl::setDeviceProperties(const std::shared_ptr<Device>& device, const DeviceProperties& deviceProperties) {
    resolvePipelineDevice(device)->setProperties(deviceProperties);
}

DeviceProperties PipelineImpl::getDeviceProperties(const std::shared_ptr<Device>& device) const {
    return resolvePipelineDevice(device)->getProperties();
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

std::vector<std::shared_ptr<Device>> PipelineImpl::getAllAssignedDevices() const {
    std::vector<std::shared_ptr<Device>> devices;
    std::unordered_set<Device*> seen;
    for(const auto& node : getAllNodes()) {
        auto device = getAssignedDevice(node);
        if(device != nullptr && seen.insert(device.get()).second) {
            devices.push_back(std::move(device));
        }
    }
    return devices;
}

std::shared_ptr<Device> PipelineImpl::registerDevice(std::shared_ptr<Device> device) {
    if(device == nullptr) {
        throw std::invalid_argument("Device is null");
    }
    // Already registered or the master itself - no-op
    if(device == defaultDevice) {
        return device;
    }
    for(const auto& registered : devices) {
        if(registered == device) {
            return device;
        }
    }
    // The device set is read from device monitor threads while running - it must not
    // grow any more (hot-plug of a new device is not supported)
    if(running) {
        throw std::runtime_error("Cannot add a device to a running pipeline");
    }
    // Two distinct Device instances connected to the same physical device would
    // produce overlapping per-device schemas - reject early
    const auto deviceId = device->getDeviceInfo().getDeviceId();
    if(!deviceId.empty()) {
        auto sameId = [&deviceId](const std::shared_ptr<Device>& other) { return other != nullptr && other->getDeviceInfo().getDeviceId() == deviceId; };
        if(sameId(defaultDevice) || std::any_of(devices.begin(), devices.end(), sameId)) {
            throw std::invalid_argument(fmt::format("A different Device instance with id '{}' is already part of the pipeline", deviceId));
        }
    }

    // First device is promoted to master so getDefaultDevice() and nodes created
    // without an explicit device keep working
    if(defaultDevice == nullptr) {
        defaultDevice = device;
    } else {
        devices.push_back(device);
    }
    return device;
}

std::vector<std::shared_ptr<Device>> PipelineImpl::getDevices() const {
    std::vector<std::shared_ptr<Device>> result;
    if(defaultDevice != nullptr) {
        result.push_back(defaultDevice);
    }
    result.insert(result.end(), devices.begin(), devices.end());
    return result;
}

std::shared_ptr<Device> PipelineImpl::getInputSourceDevice(const Node::Input* input) const {
    auto throwIfAmbiguous = [input](std::size_t distinctSources) {
        if(distinctSources > 1) {
            throw std::runtime_error(fmt::format("Input '{}' receives data from sources on more than one device", input->toString()));
        }
    };
    if(isBuild) {
        // Resolved from the user graph at build(), before bridge insertion
        auto it = inputSourceDevices.find(input);
        if(it == inputSourceDevices.end()) {
            return nullptr;
        }
        throwIfAmbiguous(it->second.size());
        return it->second.front();
    }
    // Not built yet - resolve live; the graph still holds the direct user connections
    std::vector<std::shared_ptr<Device>> sources;
    for(const auto& connection : getConnectionsInternal()) {
        if(connection.in != input) continue;
        auto outNode = connection.outputNode.lock();
        std::shared_ptr<Device> device = nullptr;  // host
        if(outNode != nullptr && !outNode->runOnHost()) {
            device = getAssignedDevice(outNode);
        }
        if(std::find(sources.begin(), sources.end(), device) == sources.end()) {
            sources.push_back(device);
        }
    }
    if(sources.empty()) {
        return nullptr;
    }
    throwIfAmbiguous(sources.size());
    return sources.front();
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

// Adopt children added after the parent node is already in the pipeline.
void PipelineImpl::adoptSubtree(std::shared_ptr<Node> root) {
    if(root == nullptr) {
        return;
    }

    // Go through and modify nodes and its children
    // that they are now part of this pipeline
    std::queue<std::shared_ptr<Node>> search;
    search.push(root);
    while(!search.empty()) {
        auto curNode = search.front();
        search.pop();

        // Assign an ID to the node
        if(curNode->id == -1) {
            curNode->id = getNextUniqueId();
        }

        if(curNode->parent.lock() == nullptr) {
            curNode->parent = shared_from_this();
        } else if(curNode->parent.lock() != shared_from_this()) {
            throw std::invalid_argument("Cannot add a node that is already part of another pipeline");
        }

        // In case we have a device node without an assigned device (usually subnodes in non-DeviceNode nodes), use the default device.
        // Nodes arriving with an explicitly assigned device register it with the pipeline.
        if(auto deviceNode = std::dynamic_pointer_cast<DeviceNode>(curNode)) {
            if(deviceNode->getDevice() == nullptr) {
                deviceNode->setDevice(defaultDevice);
            } else {
                registerDevice(deviceNode->getDevice());
            }
        }

        for(auto& n : curNode->nodeMap) {
            n->parentId = curNode->id;  // Set node parent id
            search.push(n);
        }
    }
}

void PipelineImpl::add(const std::shared_ptr<Node>& node) {
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

    adoptSubtree(node);  // BFS: ids, parent weak_ptr, default device for DeviceNodes

    // Add to the map (node holds its children itself)
    nodes.push_back(node);
}

bool PipelineImpl::isRunning() const {
    return running;
}

bool PipelineImpl::isBuilt() const {
    return isBuild;
}

#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
bool PipelineImpl::hasDynamicCalibration() const {
    // call this only with locked pipelineBuildMutex
    for(const auto& node : getAllNodes()) {
        const auto nodeName = std::string_view(node->getName());
        if(nodeName == dai::node::DynamicCalibration::NAME || nodeName == dai::node::AutoCalibration::NAME) {
            return true;
        }
    }
    return false;
}
#else
bool PipelineImpl::hasDynamicCalibration() const {
    return false;
}
#endif

std::pair<std::shared_ptr<dai::node::Camera>, std::shared_ptr<dai::node::Camera>> PipelineImpl::getStereoPair(const std::shared_ptr<Device>& device) const {
    if(!device) {
        return {nullptr, nullptr};
    }
    auto stereoSockets = device->getStereoPairs();
    if(stereoSockets.size() != 1) {
        return {nullptr, nullptr};
    }
    std::pair<std::shared_ptr<dai::node::Camera>, std::shared_ptr<dai::node::Camera>> stereoPair = std::pair(nullptr, nullptr);
    // call this only with locked pipelineBuildMutex
    for(const auto& node : getAllNodes()) {
        if(std::string_view(node->getName()) == dai::node::Camera::NAME && getAssignedDevice(node) == device) {
            auto camera = std::static_pointer_cast<dai::node::Camera>(node);
            auto boardSocket = camera->getBoardSocket();
            if(boardSocket == stereoSockets[0].left) {
                stereoPair.first = camera;
            } else if(boardSocket == stereoSockets[0].right) {
                stereoPair.second = camera;
            }
        }
    }
    return stereoPair;
}

void PipelineImpl::build() {
    std::unique_lock<std::mutex> lock(pipelineBuildMutex);
    if(isBuild) return;

    // Replay setup can update runtime calibration, so run it before deriving AutoCalibration defaults.
    utility::PipelineImplHelper::setupHolisticRecordAndReplay(shared_from_this());

    // start ---Add AutoCalibration block---
#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
    const auto pipelineAutoCalibrationMode = getAutoCalibrationMode();
    const auto pipelineAutoCalibrationString = std::string(autoCalibrationModeToString(pipelineAutoCalibrationMode));
    std::string autoCalibrationString = pipelineAutoCalibrationString;
    std::optional<PipelineAutoCalibrationMode> autoCalibrationMode = pipelineAutoCalibrationMode;
    if(!autoCalibrationModeSetByApi) {
        autoCalibrationString = utility::getEnvAs<std::string>("DEPTHAI_AUTOCALIBRATION", "");
        if(!autoCalibrationString.empty()) {
            autoCalibrationMode = parseAutoCalibrationMode(autoCalibrationString);
        }
    }
    #ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    if((autoCalibrationMode == PipelineAutoCalibrationMode::CONTINUOUS || autoCalibrationMode == PipelineAutoCalibrationMode::ON_START)
       && hasDynamicCalibration()) {
        Logging::getInstance().logger.info("Pipeline contains DynamicCalibration/AutoCalibration node. Disabling implicit AutoCalibration at startup.");
        autoCalibrationMode = PipelineAutoCalibrationMode::OFF;
    }

    if(autoCalibrationMode == PipelineAutoCalibrationMode::CONTINUOUS || autoCalibrationMode == PipelineAutoCalibrationMode::ON_START) {
        if(isHostOnly()) {
            Logging::getInstance().logger.info("DEPTHAI_AUTOCALIBRATION='{}' set on host-only pipeline. Skipping AutoCalibration node creation.",
                                               autoCalibrationString);
        }
        // One AutoCalibration per device that has exactly one stereo pair with valid calibration
        for(const auto& autoCalibDevice : getAllAssignedDevices()) {
            if(!autoCalibDevice->tryGetCalibration()) {
                Logging::getInstance().logger.info("Device {} has no valid initial calibration. Skipping autocalibration.",
                                                   autoCalibDevice->getDeviceInfo().getDeviceId());
                continue;
            }
            auto stereoPair = getStereoPair(autoCalibDevice);

            auto hasStereoPairValidCalibration = [&stereoPair](const std::shared_ptr<CalibrationHandler>& calibration) -> bool {
                if(!calibration) return false;

                const auto leftSocket = stereoPair.first->getBoardSocket();
                const auto rightSocket = stereoPair.second->getBoardSocket();
                if(!calibration->hasCameraCalibration(leftSocket) || !calibration->hasCameraCalibration(rightSocket)) {
                    return false;
                }

                try {
                    // getDefaultIntrinsics() validates intrinsic matrix shape/content and throws on invalid data.
                    calibration->getDefaultIntrinsics(leftSocket);
                    calibration->getDefaultIntrinsics(rightSocket);
                    calibration->getCameraExtrinsics(leftSocket, rightSocket);
                    return true;
                } catch(const std::exception& ex) {
                    return false;
                }
            };

            if(stereoPair.first && stereoPair.second && hasStereoPairValidCalibration(autoCalibDevice->tryGetCalibration())) {
                auto autoCalibrationNode = create<dai::node::AutoCalibration>(shared_from_this(), autoCalibDevice)->build(stereoPair.first, stereoPair.second);
                Logging::getInstance().logger.info("AutoCalibration is initialized");

                // Build-time flash safety: disable flashing when runtime calibration differs from EEPROM.
                const auto runtimeCalibration = autoCalibDevice->tryGetCalibration();
                bool allowFlashCalibration = autoCalibrationNode->initialConfig->flashCalibration;
                if(allowFlashCalibration && runtimeCalibration) {
                    try {
                        const auto eepromCalibration = autoCalibDevice->readFactoryCalibration();

                        bool compared = false;
                        for(const auto socket : {stereoPair.first->getBoardSocket(), stereoPair.second->getBoardSocket()}) {
                            if(socket == CameraBoardSocket::AUTO) {
                                continue;
                            }
                            compared = true;
                            if(hasDifferentDistortion(*runtimeCalibration, eepromCalibration, socket)) {
                                allowFlashCalibration = false;
                                Logging::getInstance().logger.info(
                                    "AutoCalibration build-time flash safety: runtime calibration differs from EEPROM on socket {}. Disabling "
                                    "flashCalibration.",
                                    static_cast<int>(socket));
                                break;
                            }
                        }

                        if(!compared) {
                            allowFlashCalibration = false;
                            Logging::getInstance().logger.info(
                                "AutoCalibration build-time flash safety: no valid stereo sockets to compare. Disabling flashCalibration.");
                        }
                    } catch(const std::exception& ex) {
                        allowFlashCalibration = false;
                        Logging::getInstance().logger.info(
                            "AutoCalibration build-time flash safety: failed to read EEPROM calibration ({}). Disabling flashCalibration.", ex.what());
                    }
                }
                autoCalibrationNode->initialConfig->flashCalibration = allowFlashCalibration;

                if(autoCalibrationMode == PipelineAutoCalibrationMode::CONTINUOUS) {
                    autoCalibrationNode->initialConfig->mode = dai::AutoCalibrationConfig::Mode::CONTINUOUS;
                } else {
                    autoCalibrationNode->initialConfig->mode = dai::AutoCalibrationConfig::Mode::ON_START;
                }
            }
        }
    } else if(!autoCalibrationMode && !autoCalibrationString.empty()) {
        Logging::getInstance().logger.warn("DEPTHAI_AUTOCALIBRATION can be CONTINUOUS, ON_START or OFF not {}", autoCalibrationString);
    }
    #endif
#endif
    // end of ---Add AutoCalibration block---

    // Run first build stage for all nodes
    for(const auto& node : getAllNodes()) {
        node->buildStage1();
    }

    // Go through the build stages sequentially
    for(const auto& node : getAllNodes()) {
        node->buildStage2();
    }

    for(const auto& node : getAllNodes()) {
        node->buildStage3();
    }

    if(getAllAssignedDevices().size() > 1 && (enablePipelineDebugging || utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false))) {
        throw std::runtime_error("Pipeline debugging is not supported for multi-device pipelines in this MVP");
    }

    utility::PipelineImplHelper::setupPipelineDebuggingPre(shared_from_this());

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

    // Resolve every input's source device from the user graph, before bridge
    // insertion rewires connections through XLink bridge nodes
    inputSourceDevices.clear();
    for(auto& connection : getConnectionsInternal()) {
        auto outNode = connection.outputNode.lock();
        std::shared_ptr<Device> sourceDevice = nullptr;  // host
        if(outNode != nullptr && !outNode->runOnHost()) {
            sourceDevice = getAssignedDevice(outNode);
        }
        auto& sources = inputSourceDevices[connection.in];
        if(std::find(sources.begin(), sources.end(), sourceDevice) == sources.end()) {
            sources.push_back(sourceDevice);
        }
    }

    // Create a map of already visited nodes to only create one xlink bridge
    std::unordered_map<dai::Node::Output*, dai::node::internal::XLinkOutBridge> bridgesOut;
    std::unordered_map<dai::Node::Input*, dai::node::internal::XLinkInBridge> bridgesIn;
    std::unordered_set<std::string> uniqueStreamNames;
    xlinkBridges.clear();
    bridgeHostDevices.clear();
    fatalDevices.clear();
    for(auto& connection : getConnectionsInternal()) {
        auto inNode = connection.inputNode.lock();
        auto outNode = connection.outputNode.lock();
        if(!inNode || !outNode) {
            throw std::runtime_error(fmt::format(
                "Input node in connection {}-{}_{}-{} is null", connection.inputName, connection.inputGroup, connection.outputName, connection.outputGroup));
        }
        if(!outNode->runOnHost() && !inNode->runOnHost()) {
            auto outDevice = getAssignedDevice(outNode);
            auto inDevice = getAssignedDevice(inNode);
            if(outDevice == nullptr || inDevice == nullptr) {
                throw std::runtime_error("Device-to-device connection found with an unassigned device");
            }
            if(outDevice != inDevice) {
                // Device A -> device B: relay through the host. A-side bridge is shared
                // with any direct host consumers of the same output, B-side with any
                // host producers into the same input.
                if(bridgesOut.count(connection.out) == 0) {
                    bridgesOut[connection.out] = dai::node::internal::XLinkOutBridge{
                        create<node::internal::XLinkOut>(shared_from_this(), outDevice),
                        create<node::internal::XLinkInHost>(shared_from_this()),
                    };
                    auto& outBridge = bridgesOut[connection.out];
                    auto streamName = fmt::format("__x_{}_{}_{}", outNode->id, connection.outputGroup, connection.outputName);
                    if(uniqueStreamNames.count(streamName) > 0) {
                        throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                    }
                    uniqueStreamNames.insert(streamName);
                    outBridge.xLinkOut->setStreamName(streamName);
                    outBridge.xLinkInHost->setStreamName(streamName);
                    outBridge.xLinkInHost->setConnection(outDevice->getConnection());
                    connection.out->link(outBridge.xLinkOut->input);
                    bridgeHostDevices[outBridge.xLinkInHost->id] = outDevice;
                    xlinkBridges.push_back({outBridge.xLinkOut->id, outBridge.xLinkInHost->id});
                    connection.out->xLinkBridge = std::make_shared<dai::node::internal::XLinkOutBridge>(outBridge);
                }
                if(bridgesIn.count(connection.in) == 0) {
                    bridgesIn[connection.in] = dai::node::internal::XLinkInBridge{
                        create<node::internal::XLinkOutHost>(shared_from_this()),
                        create<node::internal::XLinkIn>(shared_from_this(), inDevice),
                    };
                    auto& inBridge = bridgesIn[connection.in];
                    auto streamName = fmt::format("__x_{}_{}_{}", inNode->id, connection.inputGroup, connection.inputName);
                    if(uniqueStreamNames.count(streamName) > 0) {
                        throw std::runtime_error(fmt::format("Stream name '{}' is not unique", streamName));
                    }
                    uniqueStreamNames.insert(streamName);
                    inBridge.xLinkOutHost->setStreamName(streamName);
                    inBridge.xLinkIn->setStreamName(streamName);
                    inBridge.xLinkOutHost->setConnection(inDevice->getConnection());
                    inBridge.xLinkIn->out.link(*connection.in);
                    inBridge.xLinkOutHost->allowStreamResize(true);
                    // Relay queue: drop rather than block when the consuming device lags
                    inBridge.xLinkOutHost->in.setBlocking(false);
                    inBridge.xLinkOutHost->in.setMaxSize(8);
                    bridgeHostDevices[inBridge.xLinkOutHost->id] = inDevice;
                    xlinkBridges.push_back({inBridge.xLinkOutHost->id, inBridge.xLinkIn->id});
                    connection.in->xLinkBridge = std::make_shared<dai::node::internal::XLinkInBridge>(inBridge);
                }
                auto outBridge = bridgesOut[connection.out];
                auto inBridge = bridgesIn[connection.in];
                connection.out->unlink(*connection.in);
                outBridge.xLinkInHost->out.link(inBridge.xLinkOutHost->in);
                // The consuming device depends on another device's stream - losing it
                // for good stops the pipeline (see onDeviceStateChanged)
                fatalDevices.insert(inDevice.get());
                Logging::getInstance().logger.info("Inserted host relay '__x_{}_{}_{}': device {} -> device {}",
                                                   outNode->id,
                                                   connection.outputGroup,
                                                   connection.outputName,
                                                   outDevice->getDeviceInfo().getDeviceId(),
                                                   inDevice->getDeviceInfo().getDeviceId());
            }
        } else if(!outNode->runOnHost() && inNode->runOnHost()) {
            auto outDevice = getAssignedDevice(outNode);
            if(outDevice == nullptr) {
                throw std::runtime_error(fmt::format("Device node '{}' has no assigned device", outNode->getName()));
            }
            // Check if the bridge already exists
            if(bridgesOut.count(connection.out) == 0) {  // If the bridge does not already exist, create one
                // // Create a new bridge
                bridgesOut[connection.out] = dai::node::internal::XLinkOutBridge{
                    create<node::internal::XLinkOut>(shared_from_this(), outDevice),
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
                xLinkBridge.xLinkInHost->setConnection(outDevice->getConnection());
                connection.out->link(xLinkBridge.xLinkOut->input);
                bridgeHostDevices[xLinkBridge.xLinkInHost->id] = outDevice;

                // Note the created bridge for serialization (for visualization)
                xlinkBridges.push_back({xLinkBridge.xLinkOut->id, xLinkBridge.xLinkInHost->id});
                // Store the bridge in the Output object
                connection.out->xLinkBridge = std::make_shared<dai::node::internal::XLinkOutBridge>(xLinkBridge);
            }
            auto xLinkBridge = bridgesOut[connection.out];
            connection.out->unlink(*connection.in);  // Unlink the connection
            xLinkBridge.xLinkInHost->out.link(*connection.in);
        } else if(!inNode->runOnHost() && outNode->runOnHost()) {
            auto inDevice = getAssignedDevice(inNode);
            if(inDevice == nullptr) {
                throw std::runtime_error(fmt::format("Device node '{}' has no assigned device", inNode->getName()));
            }
            // Check if the bridge already exists
            if(bridgesIn.count(connection.in) == 0) {  // If the bridge does not already exist, create one
                // // Create a new bridge
                bridgesIn[connection.in] = dai::node::internal::XLinkInBridge{
                    create<node::internal::XLinkOutHost>(shared_from_this()),
                    create<node::internal::XLinkIn>(shared_from_this(), inDevice),
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
                xLinkBridge.xLinkOutHost->setConnection(inDevice->getConnection());
                xLinkBridge.xLinkIn->out.link(*connection.in);
                xLinkBridge.xLinkOutHost->allowStreamResize(true);
                bridgeHostDevices[xLinkBridge.xLinkOutHost->id] = inDevice;

                // Note the created bridge for serialization (for visualization)
                xlinkBridges.push_back({xLinkBridge.xLinkOutHost->id, xLinkBridge.xLinkIn->id});
                // Store the bridge in the Input object
                connection.in->xLinkBridge = std::make_shared<dai::node::internal::XLinkInBridge>(xLinkBridge);
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
                auto outputDevice = getAssignedDevice(node);
                if(outputDevice == nullptr) {
                    throw std::runtime_error(fmt::format("Device node '{}' has no assigned device", node->getName()));
                }
                // For every queue connection, if it's connected to a device node, create a bridge, if it doesn't exist
                if(bridgesOut.count(queueConnection.output) == 0) {
                    // // Create a new bridge
                    bridgesOut[queueConnection.output] = dai::node::internal::XLinkOutBridge{
                        create<node::internal::XLinkOut>(shared_from_this(), outputDevice),
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
                    xLinkBridge.xLinkInHost->setConnection(outputDevice->getConnection());
                    queueConnection.output->link(xLinkBridge.xLinkOut->input);
                    bridgeHostDevices[xLinkBridge.xLinkInHost->id] = outputDevice;

                    // Note the created bridge for serialization (for visualization)
                    xlinkBridges.push_back({xLinkBridge.xLinkOut->id, xLinkBridge.xLinkInHost->id});
                    // Store the bridge in the Output object
                    queueConnection.output->xLinkBridge = std::make_shared<dai::node::internal::XLinkOutBridge>(xLinkBridge);
                }
                auto xLinkBridge = bridgesOut[queueConnection.output];
                queueConnection.output->unlink(queueConnection.queue);  // Unlink the original connection
                xLinkBridge.xLinkInHost->out.link(queueConnection.queue);
            }
        }
    }

    utility::PipelineImplHelper::setupPipelineDebuggingPost(shared_from_this(), bridgesOut, bridgesIn);

    isBuild = true;
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

    // Starts pipeline, go through all nodes and start them
    // Implicitly build (if not already)
    build();

    for(const auto& node : getAllNodes()) {
        node->postBuildStage();
    }

    if(Logging::getInstance().logger.should_log(spdlog::level::debug)) {
        Logging::getInstance().logger.debug("Full schema dump: {}", ((nlohmann::json)getPipelineSchema(SerializationType::JSON, false)).dump());
    }

    const auto devices = getAllAssignedDevices();
    // Add pointer to the pipeline to every device before device-side startup, so that
    // device-side telemetry can resolve the pipeline and a disconnect during another
    // device's (slow) startup still finds the pipeline for reconnection.
    {
        const auto weak = std::weak_ptr<PipelineImpl>(shared_from_this());
        if(defaultDevice) defaultDevice->pipelinePtr = weak;
        for(const auto& device : devices) {
            device->pipelinePtr = weak;
        }
    }

    // Start device pipeline if not host-only
    if(!isHostOnly()) {
        DAI_CHECK_V(!devices.empty(), "No devices are assigned to device nodes");
        // Start all devices in parallel, all-or-nothing: if any fails, close the
        // ones that started and rethrow
        std::vector<std::thread> startThreads;
        std::vector<std::exception_ptr> startErrors(devices.size());
        // Each thread writes only its own index; read after join
        std::vector<uint8_t> started(devices.size(), 0);
        for(std::size_t i = 0; i < devices.size(); i++) {
            startThreads.emplace_back([this, &devices, &startErrors, &started, i]() {
                try {
                    devices[i]->startPipeline(Pipeline(shared_from_this()));
                    started[i] = true;
                } catch(...) {
                    startErrors[i] = std::current_exception();
                }
            });
        }
        for(auto& thread : startThreads) {
            thread.join();
        }
        for(std::size_t i = 0; i < devices.size(); i++) {
            if(startErrors[i]) {
                for(std::size_t j = 0; j < devices.size(); j++) {
                    if(started[j]) devices[j]->close();
                }
                std::rethrow_exception(startErrors[i]);
            }
        }
    }

    // All devices that have no recorded transition are up. A monitor thread may have
    // recorded a transition already (pipelinePtr is set before the start loop), and a
    // terminal FAILED recorded in that window must not be clobbered - handle it below.
    std::vector<std::shared_ptr<Device>> failedDuringStart;
    {
        std::lock_guard<std::mutex> stateLock(deviceStateMtx);
        for(const auto& device : devices) {
            auto it = deviceStates.find(device.get());
            if(it == deviceStates.end()) {
                deviceStates[device.get()] = DeviceState::RUNNING;
            } else if(it->second == DeviceState::FAILED) {
                failedDuringStart.push_back(device);
            }
        }
    }

    // Indicate that pipeline is running
    running = true;

    // A device that died for good while the others were still starting gets the full
    // failure handling now (idle its streams, fatal/last-device stop decision)
    for(const auto& device : failedDuringStart) {
        onDeviceStateChanged(device.get(), DeviceState::FAILED);
    }

    // Starts pipeline, go through all nodes and start them
    for(const auto& node : getAllNodes()) {
        if(node->runOnHost()) {
            node->start();
        }
    }

    telemetryPipelineStartedAt = std::chrono::steady_clock::now();

    const auto pipeline = Pipeline(shared_from_this());
    const auto telemetrySchema = makeTelemetrySchemaJson(getPipelineSchema(SerializationType::JSON, false));
    emitTelemetryNodeCreatedEvents(pipeline, telemetrySchema);
    dai::utility::Telemetry::getInstance().event(pipeline,
                                                 "depthai_pipeline_start",
                                                 nlohmann::json{
                                                     {"host_only", isHostOnly()},
                                                     {"pipeline_schema", telemetrySchema},
                                                 });

    // Setup pipeline state trace logging if enabled
    if(buildingOnHost && utility::getEnvAs<bool>("DEPTHAI_PIPELINE_DEBUGGING", false)) {
        if(pipelineStateTraceOut) {
            PipelineEventAggregationConfig cfg;
            cfg.repeatIntervalSeconds = 1;
            cfg.setTimestamp(std::chrono::steady_clock::now());
            pipelineStateTraceRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));

            pipelineStateTraceOut->addCallback([](const std::shared_ptr<ADatatype>& data) {
                if(data) {
                    auto state = std::dynamic_pointer_cast<const PipelineState>(data);
                    if(state) Logging::getInstance().logger.trace("Pipeline state update: {}", state->toJson().dump());
                }
            });
        }
    }
}

void PipelineImpl::resetConnections(DeviceBase* device) {
    // rebind XLink host nodes - all of them, or only the given device's bridges
    for(auto node : getAllNodes()) {
        auto tmp = std::dynamic_pointer_cast<node::internal::XLinkInHost>(node);
        if(tmp) {
            auto it = bridgeHostDevices.find(tmp->id);
            auto bridgeDevice = it != bridgeHostDevices.end() ? it->second : defaultDevice;
            if(device == nullptr || (bridgeDevice != nullptr && bridgeDevice.get() == device)) {
                if(bridgeDevice == nullptr || bridgeDevice->getConnection() == nullptr) throw std::runtime_error("Connection lost");
                tmp->setConnection(bridgeDevice->getConnection());
            }
        }
        auto tmp2 = std::dynamic_pointer_cast<node::internal::XLinkOutHost>(node);
        if(tmp2) {
            auto it = bridgeHostDevices.find(tmp2->id);
            auto bridgeDevice = it != bridgeHostDevices.end() ? it->second : defaultDevice;
            if(device == nullptr || (bridgeDevice != nullptr && bridgeDevice.get() == device)) {
                if(bridgeDevice == nullptr || bridgeDevice->getConnection() == nullptr) throw std::runtime_error("Connection lost");
                tmp2->setConnection(bridgeDevice->getConnection());
            }
        }
    }

    // re-send the pipeline - only to the reconnected device when given
    if(!isHostOnly()) {
        for(const auto& assignedDevice : getAllAssignedDevices()) {
            if(device == nullptr || assignedDevice.get() == device) {
                assignedDevice->startPipeline(Pipeline(shared_from_this()));
            }
        }
    }
}

void PipelineImpl::disconnectXLinkHosts(DeviceBase* device) {
    // make the (given device's) XLink host nodes exit instead of waiting to reconnect
    for(auto node : getAllNodes()) {
        auto tmp = std::dynamic_pointer_cast<node::internal::XLinkInHost>(node);
        if(tmp) {
            auto it = bridgeHostDevices.find(tmp->id);
            auto bridgeDevice = it != bridgeHostDevices.end() ? it->second : defaultDevice;
            if(device == nullptr || (bridgeDevice != nullptr && bridgeDevice.get() == device)) tmp->disconnect();
        }
        auto tmp2 = std::dynamic_pointer_cast<node::internal::XLinkOutHost>(node);
        if(tmp2) {
            auto it = bridgeHostDevices.find(tmp2->id);
            auto bridgeDevice = it != bridgeHostDevices.end() ? it->second : defaultDevice;
            if(device == nullptr || (bridgeDevice != nullptr && bridgeDevice.get() == device)) tmp2->disconnect();
        }
    }
}

void PipelineImpl::onDeviceStateChanged(DeviceBase* device, DeviceState state) {
    // Find the shared_ptr of the reporting device among this pipeline's devices
    std::shared_ptr<Device> devicePtr;
    for(const auto& d : getAllAssignedDevices()) {
        if(static_cast<DeviceBase*>(d.get()) == device) {
            devicePtr = d;
            break;
        }
    }
    if(devicePtr == nullptr) {
        for(const auto& d : getDevices()) {
            if(static_cast<DeviceBase*>(d.get()) == device) {
                devicePtr = d;
                break;
            }
        }
    }
    if(devicePtr == nullptr) return;

    const bool pipelineRunning = running;
    std::function<void(std::shared_ptr<Device>, DeviceState)> callback;
    {
        std::lock_guard<std::mutex> lock(deviceStateMtx);
        deviceStates[device] = state;
        if(pipelineRunning) callback = deviceStateCallback;
    }
    if(state == DeviceState::FAILED) {
        // Idle this device's XLink host nodes - consumers see silence, not errors.
        // This must happen even while the pipeline is stopping: a bridge node parked
        // waiting for a reconnect is only woken by disconnect(), and stop() joins it.
        disconnectXLinkHosts(device);
    }
    if(!pipelineRunning) return;

    bool shouldStop = false;
    if(state == DeviceState::FAILED) {
        // Fatal device (consumes other devices' streams) or last device alive -> stop
        const bool fatal = fatalDevices.count(devicePtr.get()) > 0;
        bool anyAlive = false;
        {
            std::lock_guard<std::mutex> lock(deviceStateMtx);
            for(const auto& d : getAllAssignedDevices()) {
                auto it = deviceStates.find(static_cast<DeviceBase*>(d.get()));
                if(it == deviceStates.end() || it->second != DeviceState::FAILED) {
                    anyAlive = true;
                    break;
                }
            }
        }
        shouldStop = fatal || !anyAlive;
    }
    if(callback) {
        try {
            callback(devicePtr, state);
        } catch(const std::exception& ex) {
            Logging::getInstance().logger.error("Device state callback threw: {}", ex.what());
        }
    }
    if(shouldStop) {
        Logging::getInstance().logger.warn("Stopping pipeline - device {} is gone for good and was {}",
                                           devicePtr->getDeviceInfo().getDeviceId(),
                                           fatalDevices.count(devicePtr.get()) > 0 ? "a fatal device" : "the last device alive");
        // Stop from a detached thread: this runs on the device's monitor thread,
        // which stop() joins indirectly when closing the devices
        std::thread([pipeline = shared_from_this()]() {
            try {
                pipeline->stop();
            } catch(const std::exception& ex) {
                Logging::getInstance().logger.error("Failed to stop pipeline after device loss: {}", ex.what());
            }
        }).detach();
    }
}

DeviceState PipelineImpl::getDeviceState(const std::shared_ptr<Device>& device) const {
    resolvePipelineDevice(device);
    std::lock_guard<std::mutex> lock(deviceStateMtx);
    auto it = deviceStates.find(device.get());
    if(it == deviceStates.end()) {
        return DeviceState::RUNNING;  // no transition recorded yet
    }
    return it->second;
}

void PipelineImpl::setDeviceStateCallback(std::function<void(std::shared_ptr<Device>, DeviceState)> callback) {
    std::lock_guard<std::mutex> lock(deviceStateMtx);
    deviceStateCallback = std::move(callback);
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
    // Mark not running up front: device monitors report state changes while their
    // devices are closed below, and those must not re-trigger stop or user callbacks
    running = false;

    // Wake any XLink host node parked waiting for a reconnect - stopping joins the
    // node threads and a parked node would never exit on its own
    disconnectXLinkHosts();

    if(telemetryPipelineStartedAt.has_value()) {
        const auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - *telemetryPipelineStartedAt).count();
        nlohmann::json properties{
            {"host_only", isHostOnly()},
            {"pipeline_id", telemetryPipelineId},
            {"duration_ms", durationMs},
        };
        try {
            if(auto self = weak_from_this().lock()) {
                dai::utility::Telemetry::getInstance().event(Pipeline(std::move(self)), "depthai_pipeline_stop", std::move(properties));
            } else if(defaultDevice) {
                dai::utility::Telemetry::getInstance().event(*defaultDevice, "depthai_pipeline_stop", std::move(properties));
            } else {
                dai::utility::Telemetry::getInstance().event("depthai_pipeline_stop", std::move(properties));
            }
        } catch(const std::exception& ex) {
            Logging::getInstance().logger.debug("Failed to emit pipeline stop telemetry: {}", ex.what());
        }
        telemetryPipelineStartedAt.reset();
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

    // Close devices if present - a pipeline might be host only and still have a device
    // For example, one only adds host nodes. Devices close in parallel - each close
    // waits on its own watchdog/monitor threads.
    auto assignedDevices = getAllAssignedDevices();
    std::unordered_set<Device*> closedDevices;
    for(const auto& device : assignedDevices) {
        closedDevices.insert(device.get());
    }
    if(defaultDevice != nullptr && closedDevices.count(defaultDevice.get()) == 0) {
        assignedDevices.push_back(defaultDevice);
    }
    std::vector<std::thread> closeThreads;
    for(const auto& device : assignedDevices) {
        closeThreads.emplace_back([device]() { device->close(); });
    }
    for(auto& thread : closeThreads) {
        thread.join();
    }
}

PipelineImpl::~PipelineImpl() {
    stop();
    wait();

    utility::PipelineImplHelper::finishHolisticRecordAndReplay(this);
}

void PipelineImpl::run() {
    start();
    while(isRunning()) {
        processTasks(true);
    }
    wait();
}

std::vector<uint8_t> PipelineImpl::loadResource(const fs::path& uri) {
    return loadResourceCwd(uri, "/pipeline");
}

static fs::path getAbsUri(fs::path& uri, fs::path& cwd) {
    const auto uriString = pathToUtf8String(uri);
    const auto cwdString = pathToUtf8String(cwd);
    int colonLocation = uriString.find(":");
    std::string resourceType = uriString.substr(0, colonLocation + 1);
    fs::path absAssetUri;
    if(uriString[colonLocation + 1] == '/') {  // Absolute path
        absAssetUri = uri;
    } else {  // Relative path
        absAssetUri = fs::path{resourceType + cwdString + uriString.substr(colonLocation + 1)};
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
             const auto uriString = pathToUtf8String(uri);
             auto asset = p.assetManager.get(uriString);
             if(asset != nullptr) {
                 if(moveAsset) {
                     p.assetManager.remove(uriString);
                     return std::move(asset->getData());
                 }
                 return asset->getData();
             }
             for(auto& node : p.nodes) {
                 auto& assetManager = node->getAssetManager();
                 auto asset = assetManager.get(uriString);
                 if(asset != nullptr) {
                     if(moveAsset) {
                         assetManager.remove(uriString);
                         return std::move(asset->getData());
                     }
                     return asset->getData();
                 }
             }
             // Asset not found anywhere
             throw std::invalid_argument(fmt::format("No asset with key ({}) found", uri));
         }} /*, TODO (read from filesystem 'file://' or default scheme, ...) */
    };

    for(const auto& handler : protocolHandlers) {
        std::string protocolPrefix = std::string(handler.protocol) + ":";

        const auto uriString = pathToUtf8String(uri);
        if(uriString.find(protocolPrefix) == 0) {
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
                path = fs::path(pathToUtf8String(absUri).substr(protocolPrefix.size()));
            } else {
                path = fs::path(uriString.substr(protocolPrefix.size()));
            }
            return handler.handle(*this, path);
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
    if(getDefaultDevice() != nullptr) {
        getDefaultDevice()->mockCameraFeatures(pathToRecording);
    }
}

bool Pipeline::isHolisticRecordEnabled() const {
    auto envPath = utility::getEnvAs<std::string>("DEPTHAI_RECORD", "");
    return impl()->recordConfig.state == RecordConfig::RecordReplayState::RECORD
           || (!this->isBuilt() && !envPath.empty() && platform::checkPathExists(envPath, true));
}

bool Pipeline::isHolisticReplayEnabled() const {
    auto envPath = utility::getEnvAs<std::string>("DEPTHAI_REPLAY", "");
    return impl()->recordConfig.state == RecordConfig::RecordReplayState::REPLAY
           || (!this->isBuilt() && !envPath.empty() && platform::checkPathExists(envPath, false));
}

void Pipeline::enablePipelineDebugging(bool enable) {
    if(this->isBuilt()) {
        throw std::runtime_error("Cannot change pipeline debugging state after pipeline is built");
    }
    impl()->enablePipelineDebugging = enable;
}

bool Pipeline::isPipelineDebuggingEnabled() const {
    return impl()->enablePipelineDebugging;
}

std::shared_ptr<MessageQueue> Pipeline::getPipelineStateOut() const {
    return impl()->pipelineStateOut;
}
std::shared_ptr<InputQueue> Pipeline::getPipelineStateRequest() const {
    return impl()->pipelineStateRequest;
}

PipelineStateApi Pipeline::getPipelineState() {
    return impl()->getPipelineState();
}

}  // namespace dai

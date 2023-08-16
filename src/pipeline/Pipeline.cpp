#include "depthai/pipeline/Pipeline.hpp"

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/utility/Initialization.hpp"
#include "utility/spdlog-fmt.hpp"

// shared
#include "depthai-shared/pipeline/NodeConnectionSchema.hpp"

// std
#include <cassert>
#include <fstream>

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

Pipeline::Pipeline() : pimpl(std::make_shared<PipelineImpl>(*this)) {
    // // Initialize library
    // initialize();
}

Pipeline Pipeline::clone() const {
    // TODO(themarpe) - Copy assets

    Pipeline clone;

    // Make a copy of PipelineImpl
    clone.pimpl = std::make_shared<PipelineImpl>(*impl());

    // All IDs remain the same, just switch out the actual nodes with copies
    // Copy all nodes
    for(const auto& node : impl()->nodes) {
        // const auto& id = kv.first;

        // Swap out with a copy
        auto nodeClone = node->clone();
        // Set parent to be the new pipeline
        nodeClone->parent = std::weak_ptr<PipelineImpl>(clone.pimpl);

        // Add the new copy
        clone.pimpl->nodes.push_back(node->clone());
    }

    return clone;
}

Pipeline::Pipeline(std::shared_ptr<PipelineImpl> pimpl) {
    this->pimpl = pimpl;
}

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

std::vector<Node::Connection> PipelineImpl::getConnections() const {
    std::vector<Node::Connection> conns;
    auto nodeConnectionMap = getConnectionMap();
    for(const auto& kv : nodeConnectionMap) {
        const auto& connections = kv.second;
        for(const auto& conn : connections) {
            conns.push_back(conn);
        }
    }
    return conns;
}

PipelineSchema PipelineImpl::getPipelineSchema(SerializationType type) const {
    PipelineSchema schema;
    schema.globalProperties = globalProperties;

    // Loop over all nodes, and add them to schema
    for(const auto& node : getAllNodes()) {
        // const auto& node = kv.second;
        if(std::string(node->getName()) == std::string("NodeGroup")) {
            continue;
        }
        // Check if its a host node or device node
        if(node->hostNode) {
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
            node->getProperties().serialize(info.properties, type);

            // Create Io information
            auto inputs = node->getInputs();
            auto outputs = node->getOutputs();

            info.ioInfo.reserve(inputs.size() + outputs.size());

            // Add inputs
            for(const auto& input : inputs) {
                NodeIoInfo io;
                io.blocking = input.getBlocking();
                io.queueSize = input.getQueueSize();
                io.name = input.name;
                io.group = input.group;
                auto ioKey = std::make_tuple(io.group, io.name);

                io.waitForMessage = input.waitForMessage.value_or(input.defaultWaitForMessage);
                switch(input.type) {
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
                io.blocking = false;
                io.name = output.name;
                io.group = output.group;
                auto ioKey = std::make_tuple(io.group, io.name);

                switch(output.type) {
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

    std::unordered_map<NodeConnectionSchema, bool> hostDeviceXLinkBridge;
    std::unordered_map<NodeConnectionSchema, bool> deviceHostXLinkBridge;

    auto streamName = [](std::int64_t id, std::string group, std::string name) -> std::string {
        if(group == "") {
            return fmt::format("__x_{}_{}", id, name);
        } else {
            return fmt::format("__x_{}_{}[\"{}\"]", id, group, name);
        }
    };
    Node::Id xLinkBridgeId = latestId;

    auto nodeConnectionMap = getConnectionMap();
    for(const auto& kv : nodeConnectionMap) {
        const auto& connections = kv.second;

        for(const auto& conn : connections) {
            NodeConnectionSchema c;
            auto outNode = conn.outputNode.lock();
            auto inNode = conn.inputNode.lock();
            c.node1Id = outNode->id;
            c.node1Output = conn.outputName;
            c.node1OutputGroup = conn.outputGroup;
            c.node2Id = inNode->id;
            c.node2Input = conn.inputName;
            c.node2InputGroup = conn.inputGroup;

            bool outputHost = outNode->hostNode;
            bool inputHost = inNode->hostNode;

            std::shared_ptr<Node> node;

            if(outputHost && inputHost) {
                // skip - connection between host nodes doesn't have to be represented to the device
                continue;
            }

            if(outputHost == true && inputHost != true) {
                // host->device - create implicit XLinkIn node and connect it

                // Create a map entry, only one bridge is required for multiple connections
                auto xlinkConnection = c;

                auto nodeTmp = std::make_shared<node::XLinkIn>();
                nodeTmp->parent = parent.pimpl;
                nodeTmp->id = xLinkBridgeId++;
                xlinkConnection.node1Id = xLinkBridgeId;
                xlinkConnection.node1Output = nodeTmp->out.name;
                xlinkConnection.node1OutputGroup = nodeTmp->out.group;
                nodeTmp->setStreamName(streamName(c.node1Id, c.node1OutputGroup, c.node1Output));

                xlinkConnection.node2Id = 0;
                xlinkConnection.node2Input = "";
                xlinkConnection.node2InputGroup = "";

                if(hostDeviceXLinkBridge.count(xlinkConnection) <= 0) {
                    // create it
                    hostDeviceXLinkBridge[xlinkConnection] = true;
                    // and bump xlink bridge id
                    xLinkBridgeId++;

                    c.node1Id = xlinkConnection.node1Id;
                    c.node1Output = xlinkConnection.node1Output;
                    c.node1OutputGroup = xlinkConnection.node1OutputGroup;

                    node = nodeTmp;
                }

            } else if(outputHost == false && inputHost == true) {
                // device -> host

                // Create a map entry, only one bridge is required for multiple connections
                auto xlinkConnection = c;
                xlinkConnection.node2Id = 0;
                xlinkConnection.node2Input = "";
                xlinkConnection.node2InputGroup = "";

                if(deviceHostXLinkBridge.count(xlinkConnection) <= 0) {
                    // create it
                    deviceHostXLinkBridge[xlinkConnection] = true;
                    auto nodeTmp = std::make_shared<node::XLinkOut>();
                    nodeTmp->parent = parent.pimpl;
                    nodeTmp->id = xLinkBridgeId++;
                    nodeTmp->setStreamName(streamName(c.node1Id, c.node1OutputGroup, c.node1Output));

                    c.node2Id = nodeTmp->id;
                    c.node2Input = nodeTmp->input.name;
                    c.node2InputGroup = nodeTmp->input.group;
                    node = nodeTmp;
                }

            } else {
                // device->device connection
                // all set
            }

            // add the connection to the schema
            schema.connections.push_back(c);

            // Now add the created XLink bridge if necessary
            if(node) {
                // Create 'node' info
                NodeObjInfo info;
                info.id = node->id;
                info.name = node->getName();
                node->getProperties().serialize(info.properties, type);

                // Create Io information
                auto inputs = node->getInputs();
                auto outputs = node->getOutputs();

                info.ioInfo.reserve(inputs.size() + outputs.size());

                // Add inputs
                for(const auto& input : inputs) {
                    NodeIoInfo io;
                    io.blocking = input.getBlocking();
                    io.queueSize = input.getQueueSize();
                    io.name = input.name;
                    io.group = input.group;
                    auto ioKey = std::make_tuple(io.group, io.name);

                    io.waitForMessage = input.waitForMessage.value_or(input.defaultWaitForMessage);
                    switch(input.type) {
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
                    io.blocking = false;
                    io.name = output.name;
                    io.group = output.group;
                    auto ioKey = std::make_tuple(io.group, io.name);

                    switch(output.type) {
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
tl::optional<OpenVINO::Version> PipelineImpl::getRequiredOpenVINOVersion() const {
    return getPipelineOpenVINOVersion();
}

tl::optional<OpenVINO::Version> PipelineImpl::getPipelineOpenVINOVersion() const {
    // Loop over nodes, and get the required information
    tl::optional<OpenVINO::Version> version;
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
        return tl::nullopt;
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
    // Search for this node in 'nodes' vector.
    // If found, remove from vector

    // // Go through and modify nodes and its children
    // // that they are now part of this pipeline
    // std::weak_ptr<PipelineImpl> curParent;
    // std::queue<std::shared_ptr<Node>> search;
    // search.push(toRemove);
    // while(!search.empty()) {
    //     auto curNode = search.front();

    //     if(curNode->parent.lock() == nullptr) {
    //         // pass
    //     } else if(curNode->parent.lock() != parent.pimpl) {
    //         throw std::invalid_argument("Cannot remove a node that is a part of another pipeline");
    //     } else {
    //         curNode->parent = nullptr;
    //     }

    //     for(auto& n : curNode->nodeMap) {
    //         search.push(n.second);
    //     }
    // }

    // nodeMap.count(toRemove->id) {
    // }

    // // First check if node is on this pipeline (and that they are the same)
    // if(nodeMap.count(toRemove->id) > 0) {
    //     if(nodeMap.at(toRemove->id) == toRemove) {
    //         // its same object, (not same id but from different pipeline)

    //         // Steps to remove
    //         // 1. Iterate and remove this nodes output connections
    //         // 2. Remove this nodes entry in 'nodeConnectionMap'
    //         // 3. Remove node from 'nodeMap'

    //         // 1. Iterate and remove this nodes output connections
    //         for(auto& kv : nodeConnectionMap) {
    //             for(auto it = kv.second.begin(); it != kv.second.end();) {
    //                 // check if output belongs to 'toRemove' node
    //                 if(it->outputId == toRemove->id) {
    //                     // remove this connection from set
    //                     it = kv.second.erase(it);
    //                 } else {
    //                     ++it;
    //                 }
    //             }
    //         }

    //         // 2. Remove this nodes entry in 'nodeConnectionMap'
    //         nodeConnectionMap.erase(toRemove->id);

    //         // 3. Remove node from 'nodeMap'
    //         nodeMap.erase(toRemove->id);
    //     }
    // }
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
    if(out.type == Node::Output::Type::MSender && in.type == Node::Input::Type::MReceiver) return false;
    if(out.type == Node::Output::Type::SSender && in.type == Node::Input::Type::SReceiver) return false;

    // Check that datatypes match up
    for(const auto& outHierarchy : out.possibleDatatypes) {
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

void PipelineImpl::setEepromData(tl::optional<EepromData> eepromData) {
    globalProperties.calibData = eepromData;
}

tl::optional<EepromData> PipelineImpl::getEepromData() const {
    return globalProperties.calibData;
}

bool PipelineImpl::isHostOnly() const {
    // Starts pipeline, go through all nodes and start them
    bool hostOnly = true;
    for(const auto& node : nodes) {
        if(!node->hostNode) {
            hostOnly = false;
            break;
        }
    }
    return hostOnly;
}

bool PipelineImpl::isDeviceOnly() const {
    // Starts pipeline, go through all nodes and start them
    bool deviceOnly = true;
    for(const auto& node : nodes) {
        if(node->hostNode) {
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

void PipelineImpl::build() {
    // TODO(themarpe) - add mutex and set running up ahead
    if(isBuild) return;
    isBuild = true;

    // Build
    if(!isHostOnly()) {
        // throw std::invalid_argument("Pipeline contains device nodes");
        device = std::make_shared<Device>(Pipeline(shared_from_this()));
        for(auto outQ : device->getOutputQueueNames()) {
            device->getOutputQueue(outQ, 0, false);
        }
        for(auto inQ : device->getInputQueueNames()) {
            device->getInputQueue(inQ, 0, false);
        }
    }

    // Go through the build stages sequentially
    for(const auto& node : nodes) {
        node->buildStage1();
    }

    for(const auto& node : nodes) {
        node->buildStage2();
    }

    for(const auto& node : nodes) {
        node->buildStage3();
    }
}

void PipelineImpl::start() {
    // TODO(themarpe) - add mutex and set running up ahead

    // Implicitly build (if not already)
    build();

    // Indicate that pipeline is running
    running = true;

    // Starts pipeline, go through all nodes and start them
    for(const auto& node : nodes) {
        node->start();
    }
}

void PipelineImpl::wait() {
    // Waits for all nodes to finish the execution
    for(const auto& node : nodes) {
        node->wait();
    }
}

void PipelineImpl::stop() {
    // Stops the pipeline execution
    for(const auto& node : nodes) {
        node->stop();
    }

    // Indicate that pipeline is not runnin
    running = false;
}

PipelineImpl::~PipelineImpl() {
    wait();
    // TMP - might be more appropriate
    // stop();
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

}  // namespace dai

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/SideChannel.hpp"
#include "depthai/pipeline/datatype/TraceEvent.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {

Node::Node(std::unique_ptr<Properties> props, bool conf) : configureMode{conf}, propertiesHolder(std::move(props)) {}

tl::optional<OpenVINO::Version> Node::getRequiredOpenVINOVersion() {
    return tl::nullopt;
}

const Pipeline Node::getParentPipeline() const {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Pipeline Node::getParentPipeline() {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Properties& Node::getProperties() {
    return *propertiesHolder;
}

Node::Connection::Connection(Output out, Input in) {
    outputId = out.getParent().id;
    outputName = out.name;
    outputGroup = out.group;
    inputId = in.getParent().id;
    inputName = in.name;
    inputGroup = in.group;
}

Node::Connection::Connection(ConnectionInternal c) {
    auto out = c.outputNode.lock();
    auto in = c.inputNode.lock();
    if(out == nullptr || in == nullptr) {
        throw std::invalid_argument("Connection points to non existing node");
    }
    outputId = out->id;
    outputName = c.outputName;
    outputGroup = c.outputGroup;
    inputId = in->id;
    inputName = c.inputName;
    inputGroup = c.inputGroup;
}

bool Node::Connection::operator==(const Node::Connection& rhs) const {
    return (outputId == rhs.outputId && outputName == rhs.outputName && outputGroup == rhs.outputGroup && inputId == rhs.inputId && inputName == rhs.inputName
            && inputGroup == rhs.inputGroup);
}

std::string Node::Output::toString() const {
    if(group == "") {
        return fmt::format("{}", name);
    } else {
        return fmt::format("{}[\"{}\"]", group, name);
    }
}

std::string Node::Input::toString() const {
    if(group == "") {
        return fmt::format("{}", name);
    } else {
        return fmt::format("{}[\"{}\"]", group, name);
    }
}

std::vector<Node::ConnectionInternal> Node::Output::getConnections() {
    std::vector<Node::ConnectionInternal> myConnections;
    for(const auto& conn : parent.connections) {
        if(conn.outputName == name && conn.outputGroup == group) {
            myConnections.push_back(conn);
        }
    }
    return myConnections;
}

bool Node::Output::isSamePipeline(const Input& in) {
    // Check whether current output and 'in' are on same pipeline.
    // By checking parent of node
    auto outputPipeline = parent.parent.lock();
    if(outputPipeline != nullptr) {
        return (outputPipeline == in.parent.parent.lock());
    }
    return false;
}

static bool isDatatypeMatch(const Node::Output& out, const Node::Input& in) {
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
    // otherwise return false
    return false;
}

bool Node::Output::canConnect(const Input& in) {
    // Check that IoType match up
    if(type == Output::Type::MSender && in.type == Input::Type::MReceiver) return false;
    if(type == Output::Type::SSender && in.type == Input::Type::SReceiver) return false;

    // Check that datatypes match up
    if(!isDatatypeMatch(*this, in)) {
        return false;
    }

    // All checks pass
    return true;
}

void Node::Output::link(Input& in) {
    // First check if can connect
    if(!canConnect(in)) {
        throw std::runtime_error(fmt::format("Cannot link '{}.{}' to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    // Create 'Connection' object between 'out' and 'in'
    Node::ConnectionInternal connection(*this, in);

    // Check if connection was already made - the following is possible as operator[] constructs the underlying set if it doesn't exist.
    if(parent.connections.count(connection) > 0) {
        // this means a connection was already made.
        throw std::logic_error(fmt::format("'{}.{}' already linked to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    // Otherwise all is set to add a new connection
    parent.connections.insert(connection);
}

Node::ConnectionInternal::ConnectionInternal(Output& out, Input& in) {
    outputNode = out.getParent().shared_from_this();
    outputName = out.name;
    outputGroup = out.group;
    inputNode = in.getParent().shared_from_this();
    inputName = in.name;
    inputGroup = in.group;
}

bool Node::ConnectionInternal::operator==(const Node::ConnectionInternal& rhs) const {
    return (outputNode.lock() == rhs.outputNode.lock() && outputName == rhs.outputName && outputGroup == rhs.outputGroup
            && inputNode.lock() == rhs.inputNode.lock() && inputName == rhs.inputName && inputGroup == rhs.inputGroup);
}

void Node::Output::unlink(Input& in) {
    // Create 'Connection' object between 'out' and 'in'
    Node::ConnectionInternal connection(*this, in);
    if(parent.connections.count(connection) == 0) {
        // this means a connection was not present already made.
        throw std::logic_error(fmt::format("'{}.{}' not linked to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    // Unlink
    parent.connections.erase(connection);
}

void Node::Output::send(const std::shared_ptr<ADatatype>& msg) {
    for(auto& conn : getConnections()) {
        // Get node AND hold a reference to it.
        auto node = conn.inputNode.lock();
        // Safe, as long as we also hold 'node' shared_ptr
        auto inputs = node->getInputRefs();
        // Find the corresponding inputs
        for(auto& input : inputs) {
            if(input->group == conn.inputGroup && input->name == conn.inputName) {
                // Corresponding input to a given connection
                // Send the message
                input->queue.send(msg);
                using namespace std::chrono;
                auto traceEvent = std::make_shared<dai::TraceEvent>();
                RawTraceEvent rawTraceEvent;
                rawTraceEvent.dstId = input->getId();
                rawTraceEvent.srcId = getId();
                rawTraceEvent.event = RawTraceEvent::Event::SEND;
                rawTraceEvent.status = RawTraceEvent::Status::START;
                auto ts = steady_clock::now().time_since_epoch();
                rawTraceEvent.timestamp.sec = duration_cast<seconds>(ts).count();
                rawTraceEvent.timestamp.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
                traceEvent->set(rawTraceEvent);
                getParent().sideChannel->sendMessage(traceEvent);
            }
        }
    }
}

bool Node::Output::trySend(const std::shared_ptr<ADatatype>& msg) {
    bool success = true;

    for(auto& conn : getConnections()) {
        // Get node AND hold a reference to it.
        auto node = conn.inputNode.lock();
        // Safe, as long as we also hold 'node' shared_ptr
        auto inputs = node->getInputRefs();
        // Find the corresponding inputs
        for(auto& input : inputs) {
            if(input->group == conn.inputGroup && input->name == conn.inputName) {
                // Corresponding input to a given connection
                // Send the message
                success &= input->queue.trySend(msg);
            }
        }
    }

    return success;
}

void Node::Input::setBlocking(bool blocking) {
    this->blocking = blocking;
}

bool Node::Input::getBlocking() const {
    if(blocking) {
        return *blocking;
    }
    return defaultBlocking;
}

void Node::Input::setQueueSize(int size) {
    this->queueSize = size;
}

int Node::Input::getQueueSize() const {
    if(queueSize) {
        return *queueSize;
    }
    return defaultQueueSize;
}

void Node::Input::setWaitForMessage(bool waitForMessage) {
    this->waitForMessage = waitForMessage;
}

bool Node::Input::getWaitForMessage() const {
    return waitForMessage.value_or(defaultWaitForMessage);
}

void Node::Input::setReusePreviousMessage(bool waitForMessage) {
    this->waitForMessage = !waitForMessage;
}

bool Node::Input::getReusePreviousMessage() const {
    return !waitForMessage.value_or(defaultWaitForMessage);
}

const AssetManager& Node::getAssetManager() const {
    return assetManager;
}

AssetManager& Node::getAssetManager() {
    return assetManager;
}

std::vector<uint8_t> Node::loadResource(dai::Path uri) {
    std::string cwd = fmt::format("/node/{}/", id);
    return parent.lock()->loadResourceCwd(uri, cwd);
}

Node::OutputMap::OutputMap(Node& parent, std::string name, Node::Output defaultOutput) : defaultOutput(defaultOutput), name(std::move(name)) {}
Node::OutputMap::OutputMap(Node& parent, Node::Output defaultOutput) : defaultOutput(defaultOutput) {}
Node::OutputMap::OutputMap(bool ref, Node& parent, std::string name, Node::Output defaultOutput) : defaultOutput(defaultOutput), name(std::move(name)) {
    // Place oneself to the parents references
    if(ref) {
        parent.setOutputMapRefs(this);
    }
}
Node::OutputMap::OutputMap(bool ref, Node& parent, Node::Output defaultOutput) : defaultOutput(defaultOutput) {
    // Place oneself to the parents references
    if(ref) {
        parent.setOutputMapRefs(this);
    }
}
Node::Output& Node::OutputMap::operator[](const std::string& key) {
    if(count({name, key}) == 0) {
        // Create using default and rename with group and key
        Output output(defaultOutput);
        output.group = name;
        output.name = key;
        insert({{name, key}, output});
    }
    // otherwise just return reference to existing
    return at({name, key});
}
Node::Output& Node::OutputMap::operator[](std::pair<std::string, std::string> groupKey) {
    if(count(groupKey) == 0) {
        // Create using default and rename with group and key
        Output output(defaultOutput);

        // Uses \t (tab) as a special character to parse out as subgroup name
        output.group = fmt::format("{}\t{}", name, groupKey.first);
        output.name = groupKey.second;
        insert(std::make_pair(groupKey, output));
    }
    // otherwise just return reference to existing
    return at(groupKey);
}
Node::InputMap::InputMap(Node& parent, std::string name, Node::Input defaultInput) : defaultInput(defaultInput), name(std::move(name)) {}
Node::InputMap::InputMap(Node& parent, Node::Input defaultInput) : defaultInput(defaultInput) {}
Node::InputMap::InputMap(bool ref, Node& parent, std::string name, Node::Input defaultInput) : defaultInput(defaultInput), name(std::move(name)) {
    // Place oneself to the parents references
    if(ref) {
        parent.setInputMapRefs(this);
    }
}
Node::InputMap::InputMap(bool ref, Node& parent, Node::Input defaultInput) : defaultInput(defaultInput) {
    // Place oneself to the parents references
    if(ref) {
        parent.setInputMapRefs(this);
    }
}
Node::Input& Node::InputMap::operator[](const std::string& key) {
    if(count({name, key}) == 0) {
        // Create using default and rename with group and key
        Input input(defaultInput);
        input.group = name;
        input.name = key;
        insert({{name, key}, input});
    }
    // otherwise just return reference to existing
    return at({name, key});
}
Node::Input& Node::InputMap::operator[](std::pair<std::string, std::string> groupKey) {
    if(count(groupKey) == 0) {
        // Create using default and rename with group and key
        Input input(defaultInput);

        // Uses \t (tab) as a special character to parse out as subgroup name
        input.group = fmt::format("{}\t{}", name, groupKey.first);
        input.name = groupKey.second;
        insert(std::make_pair(groupKey, input));
    }
    // otherwise just return reference to existing
    return at(groupKey);
}

/// Retrieves all nodes outputs
std::vector<Node::Output> Node::getOutputs() {
    std::vector<Node::Output> result;
    auto refs = getOutputRefs();
    for(auto* x : refs) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves all nodes inputs
std::vector<Node::Input> Node::getInputs() {
    std::vector<Node::Input> result;
    auto refs = getInputRefs();
    for(auto* x : refs) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves reference to node outputs
std::vector<Node::Output*> Node::getOutputRefs() {
    std::vector<Node::Output*> tmpOutputRefs;
    // Approximate reservation
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    // Add outputRefs
    for(auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv.second);
    }
    // Add outputs from Maps
    for(auto& kvMap : outputMapRefs) {
        auto*& map = kvMap.second;
        for(auto& kv : *map) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}

/// Retrieves reference to node outputs
std::vector<const Node::Output*> Node::getOutputRefs() const {
    std::vector<const Node::Output*> tmpOutputRefs;
    // Approximate reservation
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    // Add outputRefs
    for(const auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv.second);
    }
    // Add outputs from Maps
    for(const auto& kvMap : outputMapRefs) {
        const auto* const& map = kvMap.second;
        for(const auto& kv : *map) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}
/// Retrieves reference to node inputs
std::vector<Node::Input*> Node::getInputRefs() {
    std::vector<Node::Input*> tmpInputRefs;
    // Approximate reservation
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    // Add inputRefs
    for(auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv.second);
    }
    // Add inputs from Maps
    for(auto& kvMap : inputMapRefs) {
        auto*& map = kvMap.second;
        for(auto& kv : *map) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

/// Retrieves reference to node inputs
std::vector<const Node::Input*> Node::getInputRefs() const {
    std::vector<const Node::Input*> tmpInputRefs;
    // Approximate reservation
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    // Add inputRefs
    for(const auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv.second);
    }
    // Add inputs from Maps
    for(const auto& kvMap : inputMapRefs) {
        const auto* const& map = kvMap.second;
        for(const auto& kv : *map) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

Node::Output* Node::getOutputRef(std::string name) {
    return getOutputRef("", name);
}
Node::Output* Node::getOutputRef(std::string group, std::string name) {
    auto refs = getOutputRefs();
    for(auto& out : refs) {
        if(out->group == group && out->name == name) {
            return out;
        }
    }
    return nullptr;
}

Node::Input* Node::getInputRef(std::string name) {
    return getInputRef("", name);
}
Node::Input* Node::getInputRef(std::string group, std::string name) {
    auto refs = getInputRefs();
    for(auto& input : refs) {
        if(input->group == group && input->name == name) {
            return input;
        }
    }
    return nullptr;
}

std::vector<Node::InputMap*> Node::getInputMapRefs() {
    std::vector<Node::InputMap*> tmpInputMapRefs;

    tmpInputMapRefs.reserve(inputMapRefs.size());
    // Add inputs from Maps
    for(const auto& kvMap : inputMapRefs) {
        tmpInputMapRefs.push_back(kvMap.second);
    }
    return tmpInputMapRefs;
}

std::vector<Node::OutputMap*> Node::getOutputMapRefs() {
    std::vector<Node::OutputMap*> tmpOutputMapRefs;

    tmpOutputMapRefs.reserve(outputMapRefs.size());
    // Add inputs from Maps
    for(const auto& kvMap : outputMapRefs) {
        tmpOutputMapRefs.push_back(kvMap.second);
    }
    return tmpOutputMapRefs;
}

Node::InputMap* Node::getInputMapRef(std::string group) {
    if(inputMapRefs.count(group) == 0) {
        return nullptr;
    }
    return inputMapRefs[group];
}

Node::OutputMap* Node::getOutputMapRef(std::string group) {
    if(outputMapRefs.count(group) == 0) {
        return nullptr;
    }
    return outputMapRefs[group];
}

void Node::setOutputRefs(std::initializer_list<Node::Output*> l) {
    for(auto& outRef : l) {
        outputRefs[outRef->name] = outRef;
    }
}
void Node::setOutputRefs(Node::Output* outRef) {
    outputRefs[outRef->name] = outRef;
}
void Node::setInputRefs(std::initializer_list<Node::Input*> l) {
    for(auto& inRef : l) {
        inputRefs[inRef->name] = inRef;
    }
}
void Node::setInputRefs(Node::Input* inRef) {
    inputRefs[inRef->name] = inRef;
}
void Node::setOutputMapRefs(std::initializer_list<Node::OutputMap*> l) {
    for(auto& outMapRef : l) {
        outputMapRefs[outMapRef->name] = outMapRef;
    }
}
void Node::setOutputMapRefs(Node::OutputMap* outMapRef) {
    outputMapRefs[outMapRef->name] = outMapRef;
}
void Node::setInputMapRefs(std::initializer_list<Node::InputMap*> l) {
    for(auto& inMapRef : l) {
        inputMapRefs[inMapRef->name] = inMapRef;
    }
}
void Node::setInputMapRefs(Node::InputMap* inMapRef) {
    inputMapRefs[inMapRef->name] = inMapRef;
}
void Node::buildStage1() {
    return;
};
void Node::buildStage2() {
    return;
};
void Node::buildStage3() {
    return;
};

void Node::setNodeRefs(std::initializer_list<std::pair<std::string, std::shared_ptr<Node>*>> l) {
    for(auto& nodeRef : l) {
        nodeRefs[nodeRef.first] = nodeRef.second;
    }
}
void Node::setNodeRefs(std::pair<std::string, std::shared_ptr<Node>*> nodeRef) {
    setNodeRefs({nodeRef});
}
void Node::setNodeRefs(std::string alias, std::shared_ptr<Node>* nodeRef) {
    setNodeRefs({alias, nodeRef});
}

void Node::add(std::shared_ptr<Node> node) {
    // TODO(themarpe) - check if node is already added somewhere else, etc... (as in Pipeline)
    node->parentNode = shared_from_this();

    // Add to the map (node holds its children itself)
    // nodeMap[node->id] = node;
    nodeMap.push_back(node);
}

// Recursive helpers for pipelines
Node::ConnectionMap Node::getConnectionMap() {
    ConnectionMap map;
    // self first
    map[shared_from_this()] = connections;
    // then subnodes
    for(const auto& node : nodeMap) {
        auto nodeConnMap = node->getConnectionMap();
        for(auto& kv : nodeConnMap) {
            auto& n = kv.first;
            map[n] = kv.second;
        }
    }
    return map;
}
std::shared_ptr<Node> Node::getNode(Node::Id id) {
    // Edge case
    if(this->id == id) return shared_from_this();

    // Search all nodes
    for(auto& node : nodeMap) {
        auto n = node->getNode(id);
        if(n != nullptr) {
            return n;
        }
    }
    return nullptr;
}
std::shared_ptr<const Node> Node::getNode(Node::Id id) const {
    // Edge case
    if(this->id == id) return shared_from_this();

    // Search all nodes
    for(auto& node : nodeMap) {
        auto n = node->getNode(id);
        if(n != nullptr) {
            return n;
        }
    }
    return nullptr;
}
std::vector<std::shared_ptr<Node>> Node::getAllNodes() const {
    std::vector<std::shared_ptr<Node>> nodes;
    for(auto& node : nodeMap) {
        // Add one own nodes first
        nodes.push_back(node);
        // And its subnodes
        auto n = node->getAllNodes();
        nodes.insert(nodes.end(), n.begin(), n.end());
    }
    return nodes;
}

size_t Node::ConnectionInternal::Hash::operator()(const dai::Node::ConnectionInternal& obj) const {
    size_t seed = 0;
    std::hash<std::shared_ptr<Node>> hId;
    std::hash<std::string> hStr;
    seed ^= hId(obj.outputNode.lock()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hId(obj.inputNode.lock()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}

}  // namespace dai

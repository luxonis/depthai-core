#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "spdlog/fmt/fmt.h"
#include "utility/ErrorMacros.hpp"

namespace dai {

const Pipeline Node::getParentPipeline() const {
    auto impl = parent.lock();
    if(impl == nullptr) {
        throw std::runtime_error("Pipeline is null");
    }
    return Pipeline(impl);
}

Pipeline Node::getParentPipeline() {
    auto impl = parent.lock();
    if(impl == nullptr) {
        throw std::runtime_error("Pipeline is null");
    }
    return Pipeline(impl);
}

Node::Input::~Input() = default;

Node::Connection::Connection(Output out, Input in) {
    outputId = out.getParent().id;
    outputName = out.getName();
    outputGroup = out.getGroup();
    inputId = in.getParent().id;
    inputName = in.getName();
    inputGroup = in.getGroup();
}

Node::Connection::Connection(const ConnectionInternal& c) {
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

std::string Node::createUniqueInputName() {
    std::string name = fmt::format("_{}_input_{}", getName(), inputId++);
    return name;
}
std::string Node::createUniqueOutputName() {
    std::string name = fmt::format("_{}_output_{}", getName(), outputId++);
    return name;
}

std::string Node::Output::toString() const {
    if(getGroup() == "") {
        return fmt::format("{}", getName());
    } else {
        return fmt::format("{}[\"{}\"]", getGroup(), getName());
    }
}

std::string Node::Input::toString() const {
    if(group == "") {
        return fmt::format("{}", getName());
    } else {
        return fmt::format("{}[\"{}\"]", group, getName());
    }
}

void Node::Input::setGroup(std::string newGroup) {
    group = std::move(newGroup);
}

std::string Node::Input::getGroup() const {
    return group;
}

bool Node::Input::isConnected() const {
    return !connectedOutputs.empty();
}

std::vector<Node::ConnectionInternal> Node::Output::getConnections() {
    std::vector<Node::ConnectionInternal> myConnections;
    for(const auto& conn : parent.get().connections) {
        if(conn.out == this) {
            myConnections.push_back(conn);
        }
    }
    return myConnections;
}

bool Node::Output::isSamePipeline(const Input& in) {
    auto outputPipeline = parent.get().parent.lock();
    if(outputPipeline != nullptr) {
        auto inputPipeline = in.getParent().parent.lock();
        return (outputPipeline == inputPipeline);
    }
    return false;
}

static bool isDatatypeMatch(const Node::Output& out, const Node::Input& in) {
    for(const auto& outHierarchy : out.getPossibleDatatypes()) {
        for(const auto& inHierarchy : in.possibleDatatypes) {
            if(outHierarchy.datatype == inHierarchy.datatype) return true;

            if(outHierarchy.descendants && isDatatypeSubclassOf(outHierarchy.datatype, inHierarchy.datatype)) return true;

            if(inHierarchy.descendants && isDatatypeSubclassOf(inHierarchy.datatype, outHierarchy.datatype)) return true;
        }
    }
    return false;
}

bool Node::Output::canConnect(const Input& in) {
    if(type == Output::Type::MSender && in.getType() == Input::Type::MReceiver) return false;
    if(type == Output::Type::SSender && in.getType() == Input::Type::SReceiver) return false;

    if(!isDatatypeMatch(*this, in)) {
        return false;
    }

    return true;
}

void Node::Output::link(Input& in) {
    auto outputPipeline = parent.get().parent.lock();
    auto inputPipeline = in.getParent().parent.lock();
    if(outputPipeline != nullptr && inputPipeline != nullptr && outputPipeline != inputPipeline) {
        throw std::runtime_error(fmt::format("Cannot link '{}.{}' to '{}.{}' - nodes are part of different pipelines",
                                             getParent().getName(),
                                             toString(),
                                             in.getParent().getName(),
                                             in.toString()));
    }

    if(!canConnect(in)) {
        throw std::runtime_error(fmt::format("Cannot link '{}.{}' to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    Node::ConnectionInternal connection(*this, in);

    if(parent.get().connections.count(connection) > 0) {
        throw std::logic_error(fmt::format("'{}.{}' already linked to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    parent.get().connections.insert(connection);
    connectedInputs.push_back(&in);
    in.connectedOutputs.push_back(this);
}

std::shared_ptr<dai::MessageQueue> Node::Output::createOutputQueue(unsigned int maxSize, bool blocking) {
    auto pipelinePtr = parent.get().getParentPipeline();
    if(pipelinePtr.isBuilt()) {
        throw std::runtime_error("Cannot create queue after pipeline is built");
    }
    auto queue = std::make_shared<MessageQueue>(maxSize, blocking);
    link(queue);

    pipelinePtr.impl()->outputQueues.push_back(queue);
    return queue;
}

std::shared_ptr<InputQueue> Node::Input::createInputQueue(unsigned int maxSize, bool blocking) {
    auto pipelinePtr = parent.get().getParentPipeline();
    if(pipelinePtr.isBuilt()) {
        throw std::runtime_error("Cannot create input queue after pipeline is built");
    }

    auto inputQueuePtr = std::shared_ptr<InputQueue>(new InputQueue(maxSize, blocking));  // make_shared cannot reach the private InputQueue constructor

    pipelinePtr.add(inputQueuePtr->getNode());

    inputQueuePtr->getNodeOutput().link(*this);
    return inputQueuePtr;
}

Node::ConnectionInternal::ConnectionInternal(Output& out, Input& in)
    : outputName{out.getName()}, outputGroup{out.getGroup()}, inputName{in.getName()}, inputGroup{in.getGroup()}, out{&out}, in{&in} {
    outputNode = out.getParent().shared_from_this();
    inputNode = in.getParent().shared_from_this();
}

bool Node::ConnectionInternal::operator==(const Node::ConnectionInternal& rhs) const {
    return (outputNode.lock() == rhs.outputNode.lock() && outputName == rhs.outputName && outputGroup == rhs.outputGroup
            && inputNode.lock() == rhs.inputNode.lock() && inputName == rhs.inputName && inputGroup == rhs.inputGroup);
}

void Node::Output::unlink(Input& in) {
    Node::ConnectionInternal connection(*this, in);
    if(parent.get().connections.count(connection) == 0) {
        throw std::logic_error(fmt::format("'{}.{}' not linked to '{}.{}'", getParent().getName(), toString(), in.getParent().getName(), in.toString()));
    }

    parent.get().connections.erase(connection);

    connectedInputs.erase(std::remove(connectedInputs.begin(), connectedInputs.end(), &in), connectedInputs.end());
    in.connectedOutputs.erase(std::remove(in.connectedOutputs.begin(), in.connectedOutputs.end(), this), in.connectedOutputs.end());
}

void Node::Output::send(const std::shared_ptr<ADatatype>& msg) {
    auto sendToInputs = [this, &msg]() {
        for(auto& messageQueue : connectedInputs) {
            messageQueue->send(msg);
        }
    };
    if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
        auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::OUTPUT, getName());
        sendToInputs();
    } else {
        sendToInputs();
    }
}

bool Node::Output::trySend(const std::shared_ptr<ADatatype>& msg) {
    bool success = true;

    auto sendToInputs = [this, &msg, &success]() {
        for(auto& messageQueue : connectedInputs) {
            success &= messageQueue->trySend(msg);
        }
    };
    if(pipelineEventDispatcher && pipelineEventDispatcher->sendEvents) {
        auto blockEvent = pipelineEventDispatcher->blockEvent(PipelineEvent::Type::OUTPUT, getName());
        sendToInputs();
        if(!success) {
            blockEvent.cancel();
        }
    } else {
        sendToInputs();
    }

    return success;
}

void Node::Input::setWaitForMessage(bool newWaitForMessage) {
    waitForMessage = newWaitForMessage;
}

bool Node::Input::getWaitForMessage() const {
    return waitForMessage;
}

void Node::Input::setReusePreviousMessage(bool reusePreviousMessage) {
    waitForMessage = !reusePreviousMessage;
}

bool Node::Input::getReusePreviousMessage() const {
    return !waitForMessage;
}

const AssetManager& Node::getAssetManager() const {
    return assetManager;
}

AssetManager& Node::getAssetManager() {
    return assetManager;
}

std::vector<uint8_t> Node::loadResource(const std::filesystem::path& uri) {
    std::string cwd = fmt::format("/node/{}/", id);
    return parent.lock()->loadResourceCwd(uri, cwd);
}

std::vector<uint8_t> Node::moveResource(const std::filesystem::path& uri) {
    std::string cwd = fmt::format("/node/{}/", id);
    return parent.lock()->loadResourceCwd(uri, cwd, true);
}

Node::OutputMap::OutputMap(Node& parent, std::string name, const Node::OutputDescription& defaultOutput, bool ref)
    : defaultOutput(defaultOutput), parent(parent), name(std::move(name)) {
    if(ref) {
        parent.setOutputMapRefs(this);
    }
}

Node::OutputMap::OutputMap(Node& parent, Node::OutputDescription defaultOutput, bool ref) : OutputMap(parent, "", std::move(defaultOutput), ref){};

Node::Output& Node::OutputMap::operator[](const std::string& key) {
    if(count({name, key}) == 0) {
        auto desc = defaultOutput;
        desc.group = name;
        desc.name = key;
        Output output(parent, desc, false);
        insert({{name, key}, output});
    }
    return at({name, key});
}
Node::Output& Node::OutputMap::operator[](std::pair<std::string, std::string> groupKey) {
    if(count(groupKey) == 0) {
        auto desc = defaultOutput;
        desc.group = fmt::format("{}\t{}", name, groupKey.first);  // The tab is a separator that is parsed out as the subgroup name
        desc.name = groupKey.second;
        Output output(parent, desc, false);
        insert(std::make_pair(groupKey, output));
    }
    return at(groupKey);
}

Node::InputMap::InputMap(Node& parent, std::string name, Node::InputDescription description)
    : parent(parent), defaultInput(std::move(description)), name(std::move(name)) {
    parent.setInputMapRefs(this);
}

Node::InputMap::InputMap(Node& parent, Node::InputDescription description) : InputMap(parent, "", std::move(description)){};

Node::Input& Node::InputMap::operator[](const std::string& key) {
    if(count({name, key}) == 0) {
        auto desc = defaultInput;
        desc.group = name;
        desc.name = key;
        Input input(parent, desc, false);
        insert({{name, key}, input});
    }
    return at({name, key});
}
Node::Input& Node::InputMap::operator[](std::pair<std::string, std::string> groupKey) {
    if(count(groupKey) == 0) {
        auto desc = defaultInput;
        desc.group = fmt::format("{}\t{}", name, groupKey.first);  // The tab is a separator that is parsed out as the subgroup name
        desc.name = groupKey.second;
        Input input(parent, desc, false);
        insert(std::make_pair(groupKey, input));
    }
    return at(groupKey);
}

bool Node::InputMap::has(const std::string& key) const {
    return count({name, key}) > 0;
}

std::vector<Node::Output> Node::getOutputs() {
    std::vector<Node::Output> result;
    auto refs = getOutputRefs();
    for(auto* x : refs) {
        result.push_back(*x);
    }
    return result;
}

std::vector<Node::Input> Node::getInputs() {
    std::vector<Node::Input> result;
    auto refs = getInputRefs();
    for(auto* x : refs) {
        result.push_back(*x);
    }
    return result;
}

std::vector<Node::Output*> Node::getOutputRefs() {
    std::vector<Node::Output*> tmpOutputRefs;
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    for(auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv);
    }
    for(auto& kvMap : outputMapRefs) {
        for(auto& kv : *kvMap) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}

std::vector<const Node::Output*> Node::getOutputRefs() const {
    std::vector<const Node::Output*> tmpOutputRefs;
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    for(const auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv);
    }
    for(const auto& kvMap : outputMapRefs) {
        for(const auto& kv : *kvMap) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}
std::vector<Node::Input*> Node::getInputRefs() {
    std::vector<Node::Input*> tmpInputRefs;
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    for(auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv);
    }
    for(auto* kvMap : inputMapRefs) {
        for(auto& kv : *kvMap) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

std::vector<const Node::Input*> Node::getInputRefs() const {
    std::vector<const Node::Input*> tmpInputRefs;
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    for(const auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv);
    }
    for(const auto& kvMap : inputMapRefs) {
        for(const auto& kv : *kvMap) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

Node::Output* Node::getOutputRef(const std::string& name) {
    return getOutputRef("", name);
}
Node::Output* Node::getOutputRef(const std::string& group, const std::string& name) {
    auto refs = getOutputRefs();
    for(auto& out : refs) {
        if(out->getGroup() == group && out->getName() == name) {
            return out;
        }
    }
    return nullptr;
}

Node::Input* Node::getInputRef(const std::string& name) {
    return getInputRef("", name);
}
Node::Input* Node::getInputRef(const std::string& group, const std::string& name) {
    auto refs = getInputRefs();
    for(auto& input : refs) {
        if(input->getGroup() == group && input->getName() == name) {
            return input;
        }
    }
    return nullptr;
}

std::vector<Node::InputMap*> Node::getInputMapRefs() {
    std::vector<Node::InputMap*> tmpInputMapRefs;

    tmpInputMapRefs.reserve(inputMapRefs.size());
    for(const auto& kvMap : inputMapRefs) {
        tmpInputMapRefs.push_back(kvMap);
    }
    return tmpInputMapRefs;
}

std::vector<Node::OutputMap*> Node::getOutputMapRefs() {
    std::vector<Node::OutputMap*> tmpOutputMapRefs;

    tmpOutputMapRefs.reserve(outputMapRefs.size());
    for(const auto& kvMap : outputMapRefs) {
        tmpOutputMapRefs.push_back(kvMap);
    }
    return tmpOutputMapRefs;
}

Node::InputMap* Node::getInputMapRef(const std::string& group) {
    for(auto* inMapRef : inputMapRefs) {
        if(inMapRef->name == group) {
            return inMapRef;
        }
    }
    return nullptr;
}

Node::OutputMap* Node::getOutputMapRef(const std::string& group) {
    for(auto* outMapRef : outputMapRefs) {
        if(outMapRef->name == group) {
            return outMapRef;
        }
    }
    return nullptr;
}

void Node::setOutputRefs(Node::Output* outRef) {
    outputRefs.push_back(outRef);
}
void Node::setInputRefs(Node::Input* inRef) {
    inputRefs.push_back(inRef);
}
void Node::setOutputMapRefs(Node::OutputMap* outMapRef) {
    outputMapRefs.push_back(outMapRef);
}
void Node::setInputMapRefs(Node::InputMap* inMapRef) {
    inputMapRefs.push_back(inMapRef);
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
void Node::postBuildStage() {
    return;
};

void Node::setNodeRefs(std::initializer_list<std::pair<std::string, std::shared_ptr<Node>*>> l) {
    for(auto& nodeRef : l) {
        nodeRefs.push_back(nodeRef.second);
    }
}
void Node::setNodeRefs(std::pair<std::string, std::shared_ptr<Node>*> nodeRef) {
    setNodeRefs({nodeRef});
}
void Node::setNodeRefs(std::string alias, std::shared_ptr<Node>* nodeRef) {
    setNodeRefs({alias, nodeRef});
}

void Node::add(const std::shared_ptr<Node>& node) {
    // TODO(themarpe) - check if node is already added somewhere else, etc... (as in Pipeline)
    node->parentId = this->id;
    nodeMap.push_back(node);
    if(auto pipeline = parent.lock()) {
        pipeline->adoptSubtree(node);
    }
}

void Node::remove(const std::shared_ptr<Node>& node) {
    auto pipeline = parent.lock();

    DAI_CHECK_V(pipeline != nullptr, "Pipeline is null");
    DAI_CHECK_V(!pipeline->isBuilt(), "Cannot remove node from pipeline once it is built.");

    for(auto& n : pipeline->nodes) {
        for(auto& childNode : node->nodeMap) {
            n->removeConnectionToNode(childNode);
        }
        n->removeConnectionToNode(node);
    }

    nodeMap.erase(std::remove(nodeMap.begin(), nodeMap.end(), node), nodeMap.end());
}

void Node::removeConnectionToNode(const std::shared_ptr<Node>& node) {
    for(auto it = connections.begin(); it != connections.end();) {
        if(it->inputNode.lock() == node || it->outputNode.lock() == node) {
            it = connections.erase(it);
        } else {
            ++it;
        }
    }
    for(auto& n : nodeMap) {
        n->removeConnectionToNode(node);
    }
}

bool Node::isSourceNode() const {
    return false;
}

NodeRecordParams SourceNode::getNodeRecordParams() const {
    throw std::runtime_error("Not implemented");
}

Node::Output& SourceNode::getRecordOutput() {
    throw std::runtime_error("Not implemented");
}

Node::Input& SourceNode::getReplayInput() {
    throw std::runtime_error("Not implemented");
}

Node::ConnectionMap Node::getConnectionMap() {
    ConnectionMap map;
    map[shared_from_this()] = connections;
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
    if(this->id == id) return shared_from_this();

    for(auto& node : nodeMap) {
        auto n = node->getNode(id);
        if(n != nullptr) {
            return n;
        }
    }
    return nullptr;
}
std::shared_ptr<const Node> Node::getNode(Node::Id id) const {
    if(this->id == id) return shared_from_this();

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
        nodes.push_back(node);
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

void Node::stopPipeline() {
    auto impl = parent.lock();  // Stops on a pipeline thread without a reference to it, that wait(), start() and ~Pipeline join
    if(impl != nullptr) {
        impl->stopAsync();
    }
}

void Node::Output::link(std::shared_ptr<Node> in) {
    std::cout << "Output to node linking\n" << std::flush;
    DAI_CHECK_IN(in);
    // TODO(jakgra) only call this at the build stage
    // call in correct order: from requested GUI outputs and similar to sensor outputs
    for(const auto& input : in->getRequiredInputs()) {
        if(canConnect(input.first)) {
            link(input.first);
        }
    }
}

void Node::link(std::shared_ptr<Node> in) {
    std::cout << "Node to node linking\n" << std::flush;
    DAI_CHECK_IN(in);
    // TODO(jakgra) only call this at the build stage
    // call in correct order: from requested GUI outputs and similar to sensor outputs
    for(const auto& input : in->getRequiredInputs()) {
        auto* output = requestOutput(*input.second, in->runOnHost());
        if(output) {
            output->link(input.first);
        }
    }
}

Node::Output* Node::requestOutput(const Capability& capability, bool onHost) {
    (void)capability;
    (void)onHost;
    DAI_CHECK_V(false, "Node '{}' doesn't support node to node linking. Please link outputs <--> inputs manually.", getName());
}

std::vector<std::pair<Node::Input&, std::shared_ptr<Capability>>> Node::getRequiredInputs() {
    DAI_CHECK_V(false, "Node '{}' doesn't support node to node linking. Please link outputs <--> inputs manually.", getName());
}

void Node::Output::setPossibleDatatypes(std::vector<Node::DatatypeHierarchy> types) {
    desc.types = std::move(types);
}

std::vector<Node::DatatypeHierarchy> Node::Output::getPossibleDatatypes() const {
    return desc.types;
}

void Node::Input::setPossibleDatatypes(std::vector<Node::DatatypeHierarchy> types) {
    possibleDatatypes = std::move(types);
}

std::vector<Node::DatatypeHierarchy> Node::Input::getPossibleDatatypes() const {
    return possibleDatatypes;
}

std::shared_ptr<dai::node::internal::XLinkInBridge> Node::Input::getXLinkBridge() const {
    return xLinkBridge;
}

std::shared_ptr<dai::node::internal::XLinkOutBridge> Node::Output::getXLinkBridge() const {
    return xLinkBridge;
}

std::shared_ptr<Device> Node::Input::getSourceDevice() const {
    auto pipeline = parent.get().parent.lock();
    if(pipeline == nullptr) {
        return nullptr;
    }
    const Node::Input* canonical = this;  // Keyed by the address of the node's own Input; getInputs() returns copies, so match by group and name
    for(const Node::Input* input : static_cast<const Node&>(parent.get()).getInputRefs()) {
        if(input == this) {
            canonical = this;
            break;
        }
        if(input->group == group && input->getName() == getName()) {
            canonical = input;
        }
    }
    return pipeline->getInputSourceDevice(canonical);
}

std::map<std::string, std::shared_ptr<Device>> Node::InputMap::getSourceDevices() const {
    std::map<std::string, std::shared_ptr<Device>> sourceDevices;
    for(const auto& entry : *this) {
        const auto key = entry.first.first == name ? entry.first.second : entry.first.first + "/" + entry.first.second;
        sourceDevices[key] = entry.second.getSourceDevice();
    }
    return sourceDevices;
}

}  // namespace dai

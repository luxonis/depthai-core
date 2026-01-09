#include "depthai/pipeline/PipelineStateApi.hpp"

#include "depthai/pipeline/InputQueue.hpp"

namespace dai {

PipelineState NodesStateApi::summary() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    for(auto id : nodeIds) {
        NodeEventAggregationConfig nodeCfg;
        nodeCfg.nodeId = id;
        nodeCfg.events = false;
        nodeCfg.inputs.emplace();   // Do not send any
        nodeCfg.outputs.emplace();  // Do not send any
        nodeCfg.others.emplace();   // Do not send any
        cfg.nodes.push_back(nodeCfg);
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    return *state;
}
PipelineState NodesStateApi::detailed() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    for(auto id : nodeIds) {
        NodeEventAggregationConfig nodeCfg;
        nodeCfg.nodeId = id;
        nodeCfg.events = false;
        cfg.nodes.push_back(nodeCfg);
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    return *state;
}
std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::OutputQueueState>> NodesStateApi::outputs() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    for(auto id : nodeIds) {
        NodeEventAggregationConfig nodeCfg;
        nodeCfg.nodeId = id;
        nodeCfg.inputs.emplace();  // Do not send any
        nodeCfg.others.emplace();  // Do not send any
        nodeCfg.events = false;
        cfg.nodes.push_back(nodeCfg);
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::OutputQueueState>> result;
    for(auto& [nodeId, nodeState] : state->nodeStates) {
        result[nodeId] = nodeState.outputStates;
    }
    return result;
}
std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::InputQueueState>> NodesStateApi::inputs() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    for(auto id : nodeIds) {
        NodeEventAggregationConfig nodeCfg;
        nodeCfg.nodeId = id;
        nodeCfg.outputs.emplace();  // Do not send any
        nodeCfg.others.emplace();   // Do not send any
        nodeCfg.events = false;
        cfg.nodes.push_back(nodeCfg);
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::InputQueueState>> result;
    for(auto& [nodeId, nodeState] : state->nodeStates) {
        result[nodeId] = nodeState.inputStates;
    }
    return result;
}
std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::Timing>> NodesStateApi::otherTimings() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    for(auto id : nodeIds) {
        NodeEventAggregationConfig nodeCfg;
        nodeCfg.nodeId = id;
        nodeCfg.inputs.emplace();   // Do not send any
        nodeCfg.outputs.emplace();  // Do not send any
        nodeCfg.events = false;
        cfg.nodes.push_back(nodeCfg);
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::Timing>> result;
    for(auto& [nodeId, nodeState] : state->nodeStates) {
        result[nodeId] = nodeState.otherTimings;
    }
    return result;
}

std::unordered_map<std::string, NodeState::OutputQueueState> NodeStateApi::outputs(const std::vector<std::string>& outputNames) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.outputs = outputNames;
    nodeCfg.inputs.emplace();  // Do not send any
    nodeCfg.others.emplace();  // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    std::unordered_map<std::string, NodeState::OutputQueueState> result;
    for(const auto& outputName : outputNames) {
        result[outputName] = state->nodeStates[nodeId].outputStates[outputName];
    }
    return result;
}
NodeState::OutputQueueState NodeStateApi::outputs(const std::string& outputName) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.outputs = {outputName};
    nodeCfg.inputs.emplace();  // Do not send any
    nodeCfg.others.emplace();  // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    if(state->nodeStates[nodeId].outputStates.find(outputName) == state->nodeStates[nodeId].outputStates.end()) {
        throw std::runtime_error("Output name " + outputName + " not found in NodeState for node ID " + std::to_string(nodeId));
    }
    return state->nodeStates[nodeId].outputStates[outputName];
}
std::vector<NodeState::DurationEvent> NodeStateApi::events() {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.outputs.emplace();  // Do not send any
    nodeCfg.inputs.emplace();   // Do not send any
    nodeCfg.others.emplace();   // Do not send any
    nodeCfg.events = true;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    return state->nodeStates[nodeId].events;
}
std::unordered_map<std::string, NodeState::InputQueueState> NodeStateApi::inputs(const std::vector<std::string>& inputNames) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.inputs = inputNames;
    nodeCfg.outputs.emplace();  // Do not send any
    nodeCfg.others.emplace();   // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    std::unordered_map<std::string, NodeState::InputQueueState> result;
    for(const auto& inputName : inputNames) {
        result[inputName] = state->nodeStates[nodeId].inputStates[inputName];
    }
    return result;
}
NodeState::InputQueueState NodeStateApi::inputs(const std::string& inputName) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.inputs = {inputName};
    nodeCfg.outputs.emplace();  // Do not send any
    nodeCfg.others.emplace();   // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    if(state->nodeStates[nodeId].inputStates.find(inputName) == state->nodeStates[nodeId].inputStates.end()) {
        throw std::runtime_error("Input name " + inputName + " not found in NodeState for node ID " + std::to_string(nodeId));
    }
    return state->nodeStates[nodeId].inputStates[inputName];
}
std::unordered_map<std::string, NodeState::Timing> NodeStateApi::otherTimings(const std::vector<std::string>& statNames) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.others = statNames;
    nodeCfg.outputs.emplace();  // Do not send any
    nodeCfg.inputs.emplace();   // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    std::unordered_map<std::string, NodeState::Timing> result;
    for(const auto& otherName : statNames) {
        result[otherName] = state->nodeStates[nodeId].otherTimings[otherName];
    }
    return result;
}
NodeState::Timing NodeStateApi::otherTimings(const std::string& statName) {
    PipelineEventAggregationConfig cfg;
    cfg.setTimestamp(std::chrono::steady_clock::now());
    NodeEventAggregationConfig nodeCfg;
    nodeCfg.nodeId = nodeId;
    nodeCfg.others = {statName};
    nodeCfg.outputs.emplace();  // Do not send any
    nodeCfg.inputs.emplace();   // Do not send any
    nodeCfg.events = false;
    cfg.nodes.push_back(nodeCfg);

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));
    auto state = pipelineStateOut->get<PipelineState>();
    if(!state) throw std::runtime_error("Failed to get PipelineState");
    if(state->nodeStates.find(nodeId) == state->nodeStates.end()) {
        throw std::runtime_error("Node ID " + std::to_string(nodeId) + " not found in PipelineState");
    }
    if(state->nodeStates[nodeId].otherTimings.find(statName) == state->nodeStates[nodeId].otherTimings.end()) {
        throw std::runtime_error("Stat name " + statName + " not found in NodeState for node ID " + std::to_string(nodeId));
    }
    return state->nodeStates[nodeId].otherTimings[statName];
}
void PipelineStateApi::stateAsync(std::function<void(const PipelineState&)> callback, std::optional<PipelineEventAggregationConfig> config) {
    PipelineEventAggregationConfig cfg;
    if(config.has_value()) {
        cfg = *config;
    } else {
        cfg.repeatIntervalSeconds = 1;
        cfg.setTimestamp(std::chrono::steady_clock::now());
        for(auto id : nodeIds) {
            NodeEventAggregationConfig nodeCfg;
            nodeCfg.nodeId = id;
            nodeCfg.events = false;
            cfg.nodes.push_back(nodeCfg);
        }
    }

    pipelineStateRequest->send(std::make_shared<PipelineEventAggregationConfig>(cfg));

    pipelineStateOut->addCallback([callback](const std::shared_ptr<ADatatype>& data) {
        if(data) {
            const auto state = std::dynamic_pointer_cast<const PipelineState>(data);
            if(state) callback(*state);
        }
    });
}

}  // namespace dai

#pragma once

#include "Node.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"

namespace dai {

/**
 * pipeline.getState().nodes({nodeId1}).summary() -> std::unordered_map<std::string, TimingStats>;
 * pipeline.getState().nodes({nodeId1}).detailed() -> std::unordered_map<std::string, NodeState>;
 * pipeline.getState().nodes(nodeId1).detailed() -> NodeState;
 * pipeline.getState().nodes({nodeId1}).outputs() -> std::unordered_map<std::string, TimingStats>;
 * pipeline.getState().nodes({nodeId1}).outputs({outputName1}) -> std::unordered_map<std::string, TimingStats>;
 * pipeline.getState().nodes({nodeId1}).outputs(outputName) -> TimingStats;
 * pipeline.getState().nodes({nodeId1}).events();
 * pipeline.getState().nodes({nodeId1}).inputs() -> std::unordered_map<std::string, QueueState>;
 * pipeline.getState().nodes({nodeId1}).inputs({inputName1}) -> std::unordered_map<std::string, QueueState>;
 * pipeline.getState().nodes({nodeId1}).inputs(inputName) -> QueueState;
 * pipeline.getState().nodes({nodeId1}).otherStats() -> std::unordered_map<std::string, TimingStats>;
 * pipeline.getState().nodes({nodeId1}).otherStats({statName1}) -> std::unordered_map<std::string, TimingStats>;
 * pipeline.getState().nodes({nodeId1}).outputs(statName) -> TimingStats;
 */
class NodesStateApi {
    std::vector<Node::Id> nodeIds;

    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;

   public:
    explicit NodesStateApi(std::vector<Node::Id> nodeIds, std::shared_ptr<MessageQueue> pipelineStateOut, std::shared_ptr<InputQueue> pipelineStateRequest)
        : nodeIds(std::move(nodeIds)), pipelineStateOut(pipelineStateOut), pipelineStateRequest(pipelineStateRequest) {}
    PipelineState summary();
    PipelineState detailed();
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::OutputQueueState>> outputs();
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::InputQueueState>> inputs();
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::Timing>> otherTimings();
};
class NodeStateApi {
    Node::Id nodeId;

    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;

   public:
    explicit NodeStateApi(Node::Id nodeId, std::shared_ptr<MessageQueue> pipelineStateOut, std::shared_ptr<InputQueue> pipelineStateRequest)
        : nodeId(nodeId), pipelineStateOut(pipelineStateOut), pipelineStateRequest(pipelineStateRequest) {}
    NodeState summary() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).summary().nodeStates[nodeId];
    }
    NodeState detailed() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).detailed().nodeStates[nodeId];
    }
    std::unordered_map<std::string, NodeState::OutputQueueState> outputs() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).outputs()[nodeId];
    }
    std::unordered_map<std::string, NodeState::InputQueueState> inputs() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).inputs()[nodeId];
    }
    std::unordered_map<std::string, NodeState::Timing> otherTimings() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).otherTimings()[nodeId];
    }
    std::unordered_map<std::string, NodeState::OutputQueueState> outputs(const std::vector<std::string>& outputNames);
    NodeState::OutputQueueState outputs(const std::string& outputName);
    std::vector<NodeState::DurationEvent> events();
    std::unordered_map<std::string, NodeState::InputQueueState> inputs(const std::vector<std::string>& inputNames);
    NodeState::InputQueueState inputs(const std::string& inputName);
    std::unordered_map<std::string, NodeState::Timing> otherTimings(const std::vector<std::string>& timingNames);
    NodeState::Timing otherTimings(const std::string& timingName);
};
class PipelineStateApi {
    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;
    std::vector<Node::Id> nodeIds;  // empty means all nodes

   public:
    PipelineStateApi(std::shared_ptr<MessageQueue> pipelineStateOut,
                     std::shared_ptr<InputQueue> pipelineStateRequest,
                     const std::vector<std::shared_ptr<Node>>& allNodes)
        : pipelineStateOut(std::move(pipelineStateOut)), pipelineStateRequest(std::move(pipelineStateRequest)) {
        for(const auto& n : allNodes) {
            nodeIds.push_back(n->id);
        }
    }
    NodesStateApi nodes() {
        return NodesStateApi(nodeIds, pipelineStateOut, pipelineStateRequest);
    }
    NodesStateApi nodes(const std::vector<Node::Id>& nodeIds) {
        return NodesStateApi(nodeIds, pipelineStateOut, pipelineStateRequest);
    }
    NodeStateApi nodes(Node::Id nodeId) {
        return NodeStateApi(nodeId, pipelineStateOut, pipelineStateRequest);
    }
    void stateAsync(std::function<void(const PipelineState&)> callback, std::optional<PipelineEventAggregationConfig> config = std::nullopt);
};

}  // namespace dai

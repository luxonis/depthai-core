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
    /**
     * Construct a nodes state API for a set of node ids.
     */
    explicit NodesStateApi(std::vector<Node::Id> nodeIds, std::shared_ptr<MessageQueue> pipelineStateOut, std::shared_ptr<InputQueue> pipelineStateRequest)
        : nodeIds(std::move(nodeIds)), pipelineStateOut(pipelineStateOut), pipelineStateRequest(pipelineStateRequest) {}
    /**
     * Return a summary pipeline state for the selected nodes.
     */
    PipelineState summary();
    /**
     * Return a detailed pipeline state for the selected nodes.
     */
    PipelineState detailed();
    /**
     * Return output queue state for each selected node.
     */
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::OutputQueueState>> outputs();
    /**
     * Return input queue state for each selected node.
     */
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::InputQueueState>> inputs();
    /**
     * Return timing information for each selected node.
     */
    std::unordered_map<Node::Id, std::unordered_map<std::string, NodeState::Timing>> otherTimings();
};
class NodeStateApi {
    Node::Id nodeId;

    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;

   public:
    /**
     * Construct a node state API for a specific node.
     */
    explicit NodeStateApi(Node::Id nodeId, std::shared_ptr<MessageQueue> pipelineStateOut, std::shared_ptr<InputQueue> pipelineStateRequest)
        : nodeId(nodeId), pipelineStateOut(pipelineStateOut), pipelineStateRequest(pipelineStateRequest) {}
    /**
     * Return a summary state for this node.
     */
    NodeState summary() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).summary().nodeStates[nodeId];
    }
    /**
     * Return a detailed state for this node.
     */
    NodeState detailed() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).detailed().nodeStates[nodeId];
    }
    /**
     * Return output queue states for this node.
     */
    std::unordered_map<std::string, NodeState::OutputQueueState> outputs() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).outputs()[nodeId];
    }
    /**
     * Return input queue states for this node.
     */
    std::unordered_map<std::string, NodeState::InputQueueState> inputs() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).inputs()[nodeId];
    }
    /**
     * Return timing information for this node.
     */
    std::unordered_map<std::string, NodeState::Timing> otherTimings() {
        return NodesStateApi({nodeId}, pipelineStateOut, pipelineStateRequest).otherTimings()[nodeId];
    }
    /**
     * Return output queue state for specific outputs.
     */
    std::unordered_map<std::string, NodeState::OutputQueueState> outputs(const std::vector<std::string>& outputNames);
    /**
     * Return output queue state for a specific output.
     */
    NodeState::OutputQueueState outputs(const std::string& outputName);
    /**
     * Return duration events for this node.
     */
    std::vector<NodeState::DurationEvent> events();
    /**
     * Return input queue state for specific inputs.
     */
    std::unordered_map<std::string, NodeState::InputQueueState> inputs(const std::vector<std::string>& inputNames);
    /**
     * Return input queue state for a specific input.
     */
    NodeState::InputQueueState inputs(const std::string& inputName);
    /**
     * Return timing info for specific timing names.
     */
    std::unordered_map<std::string, NodeState::Timing> otherTimings(const std::vector<std::string>& timingNames);
    /**
     * Return timing info for a specific timing name.
     */
    NodeState::Timing otherTimings(const std::string& timingName);
};
class PipelineStateApi {
    std::shared_ptr<MessageQueue> pipelineStateOut;
    std::shared_ptr<InputQueue> pipelineStateRequest;
    std::vector<Node::Id> nodeIds;  // empty means all nodes

   public:
    /**
     * Construct a pipeline state API for all nodes in a pipeline.
     */
    PipelineStateApi(std::shared_ptr<MessageQueue> pipelineStateOut,
                     std::shared_ptr<InputQueue> pipelineStateRequest,
                     const std::vector<std::shared_ptr<Node>>& allNodes)
        : pipelineStateOut(std::move(pipelineStateOut)), pipelineStateRequest(std::move(pipelineStateRequest)) {
        for(const auto& n : allNodes) {
            nodeIds.push_back(n->id);
        }
    }
    /**
     * Return a nodes state API for all nodes.
     */
    NodesStateApi nodes() {
        return NodesStateApi(nodeIds, pipelineStateOut, pipelineStateRequest);
    }
    /**
     * Return a nodes state API for a subset of nodes.
     */
    NodesStateApi nodes(const std::vector<Node::Id>& nodeIds) {
        return NodesStateApi(nodeIds, pipelineStateOut, pipelineStateRequest);
    }
    /**
     * Return a node state API for a specific node.
     */
    NodeStateApi nodes(Node::Id nodeId) {
        return NodeStateApi(nodeId, pipelineStateOut, pipelineStateRequest);
    }
    /**
     * Request state asynchronously and invoke callback when available.
     */
    void stateAsync(std::function<void(const PipelineState&)> callback, std::optional<PipelineEventAggregationConfig> config = std::nullopt);
};

}  // namespace dai

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
    /// Request duration events for this node and wait for the response.
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
    /**
     * Register a callback and request pipeline state without waiting for the response.
     * Enable pipeline debugging before starting the pipeline to receive state updates.
     *
     * Each call adds a callback retained by the state output queue. It is not
     * automatically removed after a response, even for a single request, and this
     * method does not return a callback ID for removal. Captured objects must remain
     * valid while the callback is registered.
     *
     * @param callback Called on the thread delivering each PipelineState to the shared
     * state output queue, including responses to other state requests. Return promptly;
     * do not issue blocking state queries or add/remove callbacks on that queue from
     * within the callback. The C++ state reference is valid only during the call.
     * @param config Aggregation settings. If omitted, request updates every second for
     * the nodes captured by this API, without individual duration events. An explicit
     * configuration with no repeatIntervalSeconds requests a single response.
     */
    void stateAsync(std::function<void(const PipelineState&)> callback, const std::optional<PipelineEventAggregationConfig>& config = std::nullopt);
};

}  // namespace dai

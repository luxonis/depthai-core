#include <fmt/base.h>

#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

using namespace dai;

/**
 * 1. invalid pipeline (no state yet)
 * 2. pipeline with predictable timings
 * 3. stuck pipeline (check state, full queue)
 */

class GeneratorNode : public node::CustomThreadedNode<GeneratorNode> {
    bool doStep = true;
    int runTo = 0;

    int seqNo = 0;

   public:
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    Input ping{*this, {"_ping", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output ack{*this, {"_ack", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override {
        step(0);
        while(mainLoop()) {
            step(1);
            {
                auto blockEvent = this->outputBlockEvent();
                auto msg = std::make_shared<dai::Buffer>();
                msg->sequenceNum = seqNo++;
                msg->setTimestamp(std::chrono::steady_clock::now());
                output.send(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

   private:
    void step(int seq = 0) {
        if(doStep && (runTo == seq || runTo == 0) || ping.has()) {
            // wait for ping
            auto pingMsg = ping.get<dai::Buffer>();
            doStep = pingMsg->sequenceNum >= 0;
            runTo = pingMsg->sequenceNum;
            // send ack
            auto ackMsg = std::make_shared<dai::Buffer>();
            ackMsg->sequenceNum = seq;
            ack.send(ackMsg);
        }
    }
};

class ConsumerNode : public node::CustomThreadedNode<ConsumerNode> {
    bool doStep = true;
    int runTo = 0;

   public:
    Input input{*this, {"input", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Input ping{*this, {"_ping", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output ack{*this, {"_ack", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override {
        step(0);
        volatile int sequence = 0;
        while(isRunning()) {
            this->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", sequence);
            this->pipelineEventDispatcher->startTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", ++sequence);
            std::shared_ptr<dai::Buffer> msg = nullptr;
            step(1);
            {
                auto blockEvent = this->inputBlockEvent();
                msg = input.get<dai::Buffer>();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

   private:
    void step(int seq = 0) {
        if(doStep && (runTo == seq || runTo == 0) || ping.has()) {
            // wait for ping
            auto pingMsg = ping.get<dai::Buffer>();
            doStep = pingMsg->sequenceNum >= 0;
            runTo = pingMsg->sequenceNum;
            // send ack
            auto ackMsg = std::make_shared<dai::Buffer>();
            ackMsg->sequenceNum = seq;
            ack.send(ackMsg);
        }
    }
};

class MapNode : public node::CustomThreadedNode<MapNode> {
    bool doStep = true;
    int runTo = 0;

   public:
    InputMap inputs{*this, "inputs", {DEFAULT_NAME, DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    OutputMap outputs{*this, "outputs", {DEFAULT_NAME, DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    Input ping{*this, {"_ping", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output ack{*this, {"_ack", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override {
        step(0);
        while(mainLoop()) {
            std::unordered_map<std::string, std::shared_ptr<dai::Buffer>> msg;
            step(1);
            {
                auto blockEvent = this->inputBlockEvent();
                for(auto& [name, input] : inputs) {
                    msg[name.second] = inputs[name.second].get<dai::Buffer>();
                }
            }
            step(2);
            {
                auto blockEvent = this->outputBlockEvent();
                for(auto& [name, output] : outputs) {
                    outputs[name].send(msg[name.second]);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

   private:
    void step(int seq = 0) {
        if(doStep && (runTo == seq || runTo == 0) || ping.has()) {
            // wait for ping
            auto pingMsg = ping.get<dai::Buffer>();
            doStep = pingMsg->sequenceNum >= 0;
            runTo = pingMsg->sequenceNum;
            // send ack
            auto ackMsg = std::make_shared<dai::Buffer>();
            ackMsg->sequenceNum = seq;
            ack.send(ackMsg);
        }
    }
};

class BridgeNode : public node::CustomThreadedNode<BridgeNode> {
    bool doStep = true;
    int runTo = 0;

   public:
    Input input{*this, {"input", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    Input ping{*this, {"_ping", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output ack{*this, {"_ack", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    void run() override {
        step(0);
        while(mainLoop()) {
            std::shared_ptr<dai::Buffer> msg = nullptr;
            step(1);
            {
                auto blockEvent = this->inputBlockEvent();
                msg = input.get<dai::Buffer>();
            }
            step(2);
            {
                auto blockEvent = this->outputBlockEvent();
                output.send(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

   private:
    void step(int seq = 0) {
        if(doStep && (runTo == seq || runTo == 0) || ping.has()) {
            // wait for ping
            auto pingMsg = ping.get<dai::Buffer>();
            doStep = pingMsg->sequenceNum >= 0;
            runTo = pingMsg->sequenceNum;
            // send ack
            auto ackMsg = std::make_shared<dai::Buffer>();
            ackMsg->sequenceNum = seq;
            ack.send(ackMsg);
        }
    }
};

class PipelineHandler {
    std::unordered_map<std::string, std::shared_ptr<InputQueue>> pingQueues;
    std::unordered_map<std::string, std::shared_ptr<MessageQueue>> ackQueues;
    std::unordered_map<std::string, int64_t> nodeIds;

   public:
    Pipeline pipeline;

    PipelineHandler() : pipeline(false) {
        pipeline.enablePipelineDebugging();

        auto gen1 = pipeline.create<GeneratorNode>();
        nodeIds["gen1"] = gen1->id;
        auto gen2 = pipeline.create<GeneratorNode>();
        nodeIds["gen2"] = gen2->id;
        auto bridge1 = pipeline.create<BridgeNode>();
        nodeIds["bridge1"] = bridge1->id;
        auto bridge2 = pipeline.create<BridgeNode>();
        nodeIds["bridge2"] = bridge2->id;
        auto map = pipeline.create<MapNode>();
        nodeIds["map"] = map->id;
        auto cons1 = pipeline.create<ConsumerNode>();
        nodeIds["cons1"] = cons1->id;
        auto cons2 = pipeline.create<ConsumerNode>();
        nodeIds["cons2"] = cons2->id;

        gen1->output.link(bridge1->input);
        gen2->output.link(bridge2->input);
        bridge1->output.link(map->inputs["bridge1"]);
        bridge2->output.link(map->inputs["bridge2"]);
        map->outputs["bridge1"].link(cons1->input);
        map->outputs["bridge2"].link(cons2->input);

        pingQueues["gen1"] = gen1->ping.createInputQueue();
        pingQueues["gen2"] = gen2->ping.createInputQueue();
        pingQueues["bridge1"] = bridge1->ping.createInputQueue();
        pingQueues["bridge2"] = bridge2->ping.createInputQueue();
        pingQueues["map"] = map->ping.createInputQueue();
        pingQueues["cons1"] = cons1->ping.createInputQueue();
        pingQueues["cons2"] = cons2->ping.createInputQueue();
        ackQueues["gen1"] = gen1->ack.createOutputQueue();
        ackQueues["gen2"] = gen2->ack.createOutputQueue();
        ackQueues["bridge1"] = bridge1->ack.createOutputQueue();
        ackQueues["bridge2"] = bridge2->ack.createOutputQueue();
        ackQueues["map"] = map->ack.createOutputQueue();
        ackQueues["cons1"] = cons1->ack.createOutputQueue();
        ackQueues["cons2"] = cons2->ack.createOutputQueue();
    }

    void start() {
        pipeline.start();
    }

    void stop() {
        pipeline.stop();
    }

    int ping(const std::string& nodeName, int seqNo) {
        auto pingMsg = std::make_shared<dai::Buffer>();
        pingMsg->sequenceNum = seqNo;
        pingQueues[nodeName]->send(pingMsg);
        auto ackMsg = ackQueues[nodeName]->get<dai::Buffer>();
        return ackMsg->sequenceNum;
    }

    int pingNoAck(const std::string& nodeName, int seqNo) {
        auto pingMsg = std::make_shared<dai::Buffer>();
        pingMsg->sequenceNum = seqNo;
        pingQueues[nodeName]->send(pingMsg);
        return 0;
    }

    std::vector<std::string> getNodeNames() const {
        std::vector<std::string> names;
        for(const auto& [name, _] : pingQueues) {
            names.push_back(name);
        }
        return names;
    }

    int64_t getNodeId(const std::string& nodeName) const {
        return nodeIds.at(nodeName);
    }
};

TEST_CASE("Node states test") {
    PipelineHandler ph;
    ph.start();

    // State of non-ping/ack ios should be invalid after first ping, node states should exist
    for(const auto& nodeName : ph.getNodeNames()) {
        auto ackSeq = ph.ping(nodeName, 0);
        REQUIRE(ackSeq == 0);
    }
    {
        // Nodes should now be stopped before inputs get
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto state = ph.pipeline.getPipelineState().nodes().detailed();
        for(const auto& nodeName : ph.getNodeNames()) {
            auto nodeState = state.nodeStates.at(ph.getNodeId(nodeName));
            REQUIRE(nodeState.state == dai::NodeState::State::IDLE);
            REQUIRE_FALSE(nodeState.mainLoopTiming.isValid());
            REQUIRE_FALSE(nodeState.inputsGetTiming.isValid());
            REQUIRE_FALSE(nodeState.outputsSendTiming.isValid());
            for(const auto& [inputName, inputState] : nodeState.inputStates) {
                if(inputName.rfind("_ping") != std::string::npos) continue;
                REQUIRE(inputState.state == dai::NodeState::InputQueueState::State::IDLE);
                REQUIRE_FALSE(inputState.timing.isValid());
            }
            for(const auto& [outputName, outputState] : nodeState.outputStates) {
                if(outputName.rfind("_ack") != std::string::npos) continue;
                REQUIRE(outputState.state == dai::NodeState::OutputQueueState::State::IDLE);
                REQUIRE_FALSE(outputState.timing.isValid());
            }
            for(const auto& [otherName, otherTiming] : nodeState.otherTimings) {
                REQUIRE_FALSE(otherTiming.isValid());
            }
        }
    }

    {
        // Send bridge1 to input get
        auto ackSeq = ph.ping("bridge1", 0);
        REQUIRE(ackSeq == 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge1")).detailed();
        REQUIRE(nodeState.state == dai::NodeState::State::GETTING_INPUTS);
        REQUIRE(nodeState.inputStates["input"].numQueued == 0);
        REQUIRE(nodeState.inputStates["input"].state == dai::NodeState::InputQueueState::State::WAITING);
    }
    {
        // Send gen2 to output send
        auto ackSeq = ph.ping("gen2", 0);
        REQUIRE(ackSeq == 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge2")).detailed();
        REQUIRE(nodeState.inputStates["input"].numQueued == 1);

        // Fill bridge2 input queue (currently 1/4)
        for(int i = 0; i < 3; ++i) {
            ackSeq = ph.ping("gen2", 0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge2")).detailed();
        REQUIRE(nodeState.inputStates["input"].numQueued == 4);

        // Try to send another
        ackSeq = ph.ping("gen2", 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge2")).detailed();
        REQUIRE(nodeState.inputStates["input"].numQueued == 4);
        REQUIRE(nodeState.inputStates["input"].state == dai::NodeState::InputQueueState::State::BLOCKED);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("gen2")).detailed();
        REQUIRE(nodeState.state == dai::NodeState::State::SENDING_OUTPUTS);
        REQUIRE(nodeState.outputStates["output"].state == dai::NodeState::OutputQueueState::State::SENDING);

        // Read 1 from bridge2 (unblock)
        ph.ping("bridge2", 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge2")).detailed();
        REQUIRE(nodeState.inputStates["input"].numQueued == 4);
        REQUIRE(nodeState.inputStates["input"].state == dai::NodeState::InputQueueState::State::IDLE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nodeState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("gen2")).detailed();
        REQUIRE(nodeState.outputStates["output"].state == dai::NodeState::OutputQueueState::State::IDLE);
    }

    // Let nodes run
    for(const auto& nodeName : ph.getNodeNames()) {
        ph.pingNoAck(nodeName, -1);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    {
        auto inputState = ph.pipeline.getPipelineState().nodes(ph.getNodeId("bridge2")).inputs("input");
        REQUIRE(inputState.isValid());
        REQUIRE(inputState.queueStats.maxQueued == 4);
    }

    ph.stop();
}

TEST_CASE("Node timings test") {
    PipelineHandler ph;
    ph.start();

    // Let nodes run
    for(const auto& nodeName : ph.getNodeNames()) {
        ph.ping(nodeName, -1);
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto state = ph.pipeline.getPipelineState().nodes().detailed();

    for(const auto& nodeName : ph.getNodeNames()) {
        auto nodeState = state.nodeStates.at(ph.getNodeId(nodeName));

        REQUIRE(nodeState.mainLoopTiming.isValid());
        REQUIRE(nodeState.mainLoopTiming.durationStats.averageMicrosRecent == Catch::Approx(100000).margin(50000));
        REQUIRE(nodeState.mainLoopTiming.durationStats.medianMicrosRecent == Catch::Approx(100000).margin(50000));
        REQUIRE(nodeState.mainLoopTiming.durationStats.minMicrosRecent == Catch::Approx(100000).margin(10000));
        REQUIRE(nodeState.mainLoopTiming.durationStats.minMicros == Catch::Approx(100000).margin(10000));
        REQUIRE(nodeState.mainLoopTiming.durationStats.maxMicrosRecent == Catch::Approx(150000).margin(50000));
        REQUIRE(nodeState.mainLoopTiming.durationStats.maxMicros == Catch::Approx(150000).margin(50000));

        if(nodeName.find("gen") == std::string::npos) REQUIRE(nodeState.inputsGetTiming.isValid());
        if(nodeName.find("cons") == std::string::npos) REQUIRE(nodeState.outputsSendTiming.isValid());
        for(const auto& [inputName, inputState] : nodeState.inputStates) {
            if(inputName.rfind("_ping") != std::string::npos) continue;
            REQUIRE(inputState.timing.isValid());
            REQUIRE(inputState.timing.fps == Catch::Approx(10.f).margin(5.f));
            REQUIRE(inputState.timing.durationStats.minMicros <= 0.1e6);
            REQUIRE(inputState.timing.durationStats.maxMicros <= 0.2e6);
            REQUIRE(inputState.timing.durationStats.averageMicrosRecent <= 0.2e6);
            REQUIRE(inputState.timing.durationStats.minMicrosRecent <= 0.12e6);
            REQUIRE(inputState.timing.durationStats.maxMicrosRecent <= 0.2e6);
            REQUIRE(inputState.timing.durationStats.medianMicrosRecent <= 0.2e6);
        }
        for(const auto& [outputName, outputState] : nodeState.outputStates) {
            if(outputName.rfind("_ack") != std::string::npos) continue;
            REQUIRE(outputState.timing.isValid());
            REQUIRE(outputState.timing.fps == Catch::Approx(10.f).margin(5.f));
            REQUIRE(outputState.timing.durationStats.minMicros <= 0.01e6);
            REQUIRE(outputState.timing.durationStats.maxMicros <= 0.01e6);
            REQUIRE(outputState.timing.durationStats.averageMicrosRecent <= 0.01e6);
            REQUIRE(outputState.timing.durationStats.minMicrosRecent <= 0.01e6);
            REQUIRE(outputState.timing.durationStats.maxMicrosRecent <= 0.01e6);
            REQUIRE(outputState.timing.durationStats.medianMicrosRecent <= 0.01e6);
        }
        for(const auto& [otherName, otherTiming] : nodeState.otherTimings) {
            REQUIRE(otherTiming.isValid());
        }
    }
    ph.stop();
}

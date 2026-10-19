#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <memory>
#include <thread>

#include "depthai/depthai.hpp"

// Host-only checks for Node::stopPipeline() (no device required)

namespace {
// Keeps the thread that runs the pipeline busy with a task (like a display node), then calls stopPipeline() from its own thread
class SelfStopNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, SelfStopNode> {
   public:
    constexpr static const char* NAME = "SelfStopNode";
    void run() override {
        getParentPipeline().addTask([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
        stopPipeline();
        while(mainLoop()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

// Calls stopPipeline() from a pipeline task, which runs on the thread that runs the pipeline
// (like processGroup() of a host node with sendProcessingToPipeline(true))
class TaskStopNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, TaskStopNode> {
   public:
    constexpr static const char* NAME = "TaskStopNode";
    void run() override {
        getParentPipeline().addTask([this]() {
            stopPipeline();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        });
        while(mainLoop()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

// Created last, so it is stopped last. Its slow stop() (like a device close) keeps the pipeline stop
// running after all node threads have exited.
class SlowStopNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, SlowStopNode> {
   public:
    constexpr static const char* NAME = "SlowStopNode";
    std::shared_ptr<std::atomic<bool>> destroyed = std::make_shared<std::atomic<bool>>(false);
    void run() override {
        while(mainLoop()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    void stop() override {
        ThreadedHostNode::stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    ~SlowStopNode() override {
        *destroyed = true;
    }
};
}  // namespace

TEST_CASE("stopPipeline() from a node thread: run() returns once the stop has completed") {
    std::shared_ptr<std::atomic<bool>> destroyed;
    {
        dai::Pipeline p(false);
        p.create<SelfStopNode>();
        destroyed = p.create<SlowStopNode>()->destroyed;
        p.run();
        REQUIRE_FALSE(p.isRunning());
    }
    // The stop holds no reference to the pipeline, it is destroyed with the last user reference
    REQUIRE(*destroyed);
}

TEST_CASE("stopPipeline() from a pipeline task: run() returns once the stop has completed") {
    std::shared_ptr<std::atomic<bool>> destroyed;
    {
        dai::Pipeline p(false);
        p.create<TaskStopNode>();
        destroyed = p.create<SlowStopNode>()->destroyed;
        p.run();
        REQUIRE_FALSE(p.isRunning());
    }
    REQUIRE(*destroyed);
}

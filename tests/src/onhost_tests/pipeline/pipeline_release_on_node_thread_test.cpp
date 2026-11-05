#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "depthai/depthai.hpp"

namespace {

using namespace std::chrono_literals;

struct Handshake {
    std::mutex mtx;
    std::condition_variable cv;
    bool nodeHoldsPipeline = false;
    bool nodeReleasedPipeline = false;
    bool userReleasedPipeline = false;
    bool nodeDestroyed = false;

    void set(bool Handshake::*flag) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            this->*flag = true;
        }
        cv.notify_all();
    }

    bool isSet(bool Handshake::*flag) {
        std::lock_guard<std::mutex> lock(mtx);
        return this->*flag;
    }

    bool waitFor(bool Handshake::*flag, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mtx);
        return cv.wait_for(lock, timeout, [&]() { return this->*flag; });
    }
};

// Takes a reference to its pipeline on the node thread, like nodes that call getParentPipeline() at run time.
// When holdUntilUserRelease is set, it keeps the reference until the user has released the pipeline,
// so that the node thread releases the last reference.
class PipelineHolderNode : public dai::node::CustomThreadedNode<PipelineHolderNode> {
   public:
    std::shared_ptr<Handshake> handshake;
    bool holdUntilUserRelease = true;

    ~PipelineHolderNode() override {
        if(handshake) handshake->set(&Handshake::nodeDestroyed);
    }

    void run() override {
        {
            auto pipeline = getParentPipeline();
            handshake->set(&Handshake::nodeHoldsPipeline);
            if(holdUntilUserRelease) {
                handshake->waitFor(&Handshake::userReleasedPipeline, 10s);
            }
        }
        handshake->set(&Handshake::nodeReleasedPipeline);
        while(mainLoop()) {
            std::this_thread::sleep_for(1ms);
        }
    }
};

}  // namespace

TEST_CASE("Pipeline whose last reference is released on a node thread is destroyed without a self-join") {
    auto handshake = std::make_shared<Handshake>();
    std::optional<dai::Pipeline> pipeline;
    pipeline.emplace(false);
    pipeline->create<PipelineHolderNode>()->handshake = handshake;
    pipeline->start();

    REQUIRE(handshake->waitFor(&Handshake::nodeHoldsPipeline, 10s));
    // The node thread now holds the only other reference: it releases the last one
    pipeline.reset();
    REQUIRE_FALSE(handshake->isSet(&Handshake::nodeDestroyed));
    handshake->set(&Handshake::userReleasedPipeline);

    // The pipeline stops and destroys its nodes on a separate thread
    REQUIRE(handshake->waitFor(&Handshake::nodeDestroyed, 10s));
}

TEST_CASE("Pipeline whose last reference is released on the user thread is destroyed before the release returns") {
    auto handshake = std::make_shared<Handshake>();
    std::optional<dai::Pipeline> pipeline;
    pipeline.emplace(false);
    auto node = pipeline->create<PipelineHolderNode>();
    node->handshake = handshake;
    node->holdUntilUserRelease = false;
    node.reset();
    pipeline->start();

    REQUIRE(handshake->waitFor(&Handshake::nodeReleasedPipeline, 10s));
    pipeline.reset();
    REQUIRE(handshake->isSet(&Handshake::nodeDestroyed));
}

// Child process of pipeline_release_on_node_thread_test. A node thread releases the last pipeline
// reference, then main() returns at once. The pipeline teardown is slow, so its output shows only
// when the process waits for the teardown at exit.

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "depthai/depthai.hpp"

namespace {

using namespace std::chrono_literals;

std::mutex handshakeMtx;
std::condition_variable handshakeCv;
bool nodeHoldsPipeline = false;
bool userReleasedPipeline = false;

void printLine(const char* line) {
    std::fputs(line, stdout);
    std::fputs("\n", stdout);
    std::fflush(stdout);
}

class SlowTeardownNode : public dai::node::CustomThreadedNode<SlowTeardownNode> {
   public:
    ~SlowTeardownNode() override {
        std::this_thread::sleep_for(500ms);
        printLine("pipeline destroyed");
    }

    void run() override {
        {
            auto pipeline = getParentPipeline();
            std::unique_lock<std::mutex> lock(handshakeMtx);
            nodeHoldsPipeline = true;
            handshakeCv.notify_all();
            handshakeCv.wait_for(lock, 10s, []() { return userReleasedPipeline; });
        }
        while(mainLoop()) {
            std::this_thread::sleep_for(1ms);
        }
    }
};

}  // namespace

int main() {
    std::optional<dai::Pipeline> pipeline;
    pipeline.emplace(false);
    pipeline->create<SlowTeardownNode>();
    pipeline->start();
    {
        std::unique_lock<std::mutex> lock(handshakeMtx);
        if(!handshakeCv.wait_for(lock, 10s, []() { return nodeHoldsPipeline; })) {
            printLine("node did not start");
            return 1;
        }
    }
    // The node thread now holds the only other reference: it releases the last one
    pipeline.reset();
    {
        std::lock_guard<std::mutex> lock(handshakeMtx);
        userReleasedPipeline = true;
    }
    handshakeCv.notify_all();
    printLine("main returns");
    return 0;
}

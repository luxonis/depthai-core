#include <atomic>
#include <chrono>
#include <csignal>
#include <depthai/depthai.hpp>
#include <thread>

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Create the nodes
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    // benchmarkIn->setRunOnHost(true); // The node can also run on host and include the transfer limitation, default is False
    auto* output = cam->requestFullResolutionOutput();
    output->link(benchmarkIn->input);

    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Let the logger print out the FPS
    }

    pipeline.stop();
    pipeline.wait();

    return 0;
}
#include <chrono>
#include <depthai/depthai.hpp>
#include <thread>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create the nodes
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    // benchmarkIn->setRunOnHost(true); // The node can also run on host and include the transfer limitation, default is False
    auto* output = cam->requestFullResolutionOutput();
    output->link(benchmarkIn->input);

    pipeline.start();
    while(pipeline.isRunning()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Let the logger print out the FPS
    }

    return 0;
}
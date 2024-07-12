#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/test/MyConsumer.hpp"
#include "depthai/pipeline/node/test/MyProducer.hpp"

int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;

    auto producer = pipeline.create<dai::node::test::MyProducer>();
    auto printer = pipeline.create<dai::node::test::MyConsumer>();

    producer->out.link(printer->input);

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    pipeline.stop();

    return 0;
}

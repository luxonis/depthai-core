#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create script node
        auto script = pipeline.create<dai::node::Script>();

        // Set the script content
        script->setScript(R"(
            while True:
                message = node.inputs["in"].get()
                # Or alternatively:
                # message = node.io["in"].get()
                node.warn("I received a message!")
                node.outputs["out"].send(message)
                # Or alternatively:
                # node.io["out"].send(message)
        )");

        // Create input and output queues
        auto inputQueue = script->inputs["in"].createInputQueue();
        auto outputQueue = script->outputs["out"].createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
            // Create and send a message
            auto message = std::make_shared<dai::ImgFrame>();
            std::cout << "Sending a message" << std::endl;
            inputQueue->send(message);

            // Receive the message
            auto output = outputQueue->get<dai::ImgFrame>();
            std::cout << "Received a message" << std::endl;

            // Sleep for 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
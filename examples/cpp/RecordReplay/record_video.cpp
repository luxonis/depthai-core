#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
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

int main(int argc, char** argv) {
    // Parse command line arguments
    std::string outputFile = "test_video";
    for(int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if(arg == "-o" || arg == "--output") {
            if(i + 1 < argc) {
                outputFile = argv[++i];
            }
        }
    }

    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create color camera node
        auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

        // Create video encoder node
        auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
        videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);
        cam->requestOutput(std::make_pair(1280, 720), dai::ImgFrame::Type::NV12)->link(videoEncoder->input);

        // Create record node
        auto record = pipeline.create<dai::node::RecordVideo>();
        record->setRecordVideoFile(std::filesystem::path(outputFile + ".avi"));
        videoEncoder->out.link(record->input);

        // Start pipeline
        pipeline.start();
        std::cout << "Recording video. Press Ctrl+C to stop." << std::endl;

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
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

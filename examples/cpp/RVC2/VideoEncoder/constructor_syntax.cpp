#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <fstream>
#include <chrono>
#include <thread>
#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Custom host node for saving video
class VideoSaver : public dai::node::CustomNode<VideoSaver> {
public:
    VideoSaver(dai::Node::Output& input) {
        input.link(this->inputs["video"]);
        fileHandle.open("video.h265", std::ios::binary);
        if(!fileHandle.is_open()) {
            throw std::runtime_error("Could not open video.h265 for writing");
        }
    }

    ~VideoSaver() {
        if(fileHandle.is_open()) {
            fileHandle.close();
        }
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> messageGroup) override {
        if(fileHandle.is_open()) {
            auto frame = messageGroup->get<dai::Buffer>("video");
            fileHandle.write(reinterpret_cast<const char*>(frame->getData().data()), frame->getData().size());
        }
        return nullptr;
    }

private:
    std::ofstream fileHandle;
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create camera node using constructor syntax
        auto camRgb = std::make_shared<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::make_pair(1920, 1080), 30);
        pipeline.add(camRgb);

        // Create video encoder node using constructor syntax
        auto encoder = std::make_shared<dai::node::VideoEncoder>();
        camRgb->requestOutput(std::make_pair(1920, 1080))->link(encoder->input);
        pipeline.add(encoder);

        // Create video saver node using constructor syntax
        auto saver = std::make_shared<VideoSaver>(encoder->bitstream);
        pipeline.add(saver);

        // Start pipeline
        pipeline.start();
        std::cout << "Started to save video to video.h265" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

        std::cout << "To view the encoded data, convert the stream file (.h265) into a video file (.mp4) using a command below:" << std::endl;
        std::cout << "ffmpeg -framerate 30 -i video.h265 -c copy video.mp4" << std::endl;

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 
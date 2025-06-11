#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

// Custom host node for display
class HostDisplay : public dai::node::CustomNode<HostDisplay> {
   public:
    HostDisplay() {
        sendProcessingToPipeline(false);
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> message) override {
        if(message == nullptr) return nullptr;

        auto frame = message->get<dai::ImgFrame>("frame");
        if(frame == nullptr) return nullptr;

        cv::imshow("HostDisplay", frame->getCvFrame());
        int key = cv::waitKey(1);
        if(key == 'q') {
            std::cout << "Detected 'q' - stopping the pipeline..." << std::endl;
            stopPipeline();
        }

        return nullptr;
    }
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto output = camera->requestOutput(std::make_pair(300, 300));

    // Create display node
    auto display = pipeline.create<HostDisplay>();
    output->link(display->inputs["frame"]);

    // Start pipeline
    pipeline.start();
    std::cout << "Display window opened. Press 'q' to quit." << std::endl;

    // Wait for pipeline to finish
    pipeline.wait();

    return 0;
}
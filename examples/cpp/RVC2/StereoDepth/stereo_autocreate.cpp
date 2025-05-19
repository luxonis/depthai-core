#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Custom host node for stereo visualization
class StereoVisualizer : public dai::NodeCRTP<dai::node::HostNode, StereoVisualizer> {
public:
    Input& input = inputs["in"];

    std::shared_ptr<StereoVisualizer> build(Output& out) {
        out.link(input);
        return std::static_pointer_cast<StereoVisualizer>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto frame = in->get<dai::ImgFrame>("in");
        cv::Mat frameCv = frame->getCvFrame();
        
        // Colorize the disparity map
        double minVal, maxVal;
        cv::minMaxLoc(frameCv, &minVal, &maxVal);
        
        // Normalize and convert to 8-bit
        cv::Mat normalized;
        frameCv.convertTo(normalized, CV_8UC1, 255.0 / maxVal);
        
        // Apply color map
        cv::Mat colorized;
        cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
        
        // Display
        cv::imshow("depth", colorized);
        
        if(cv::waitKey(1) == 'q') {
            stopPipeline();
        }
        return nullptr;
    }
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create stereo node with auto-created cameras
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        stereo->build(true); // autoCreateCameras = true

        // Create and configure visualizer node
        auto visualizer = pipeline.create<StereoVisualizer>()->build(stereo->disparity);

        // Start pipeline
        pipeline.start();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 
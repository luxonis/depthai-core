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
        
        // Normalize for better visualization
        cv::Mat normalized;
        frameCv.convertTo(normalized, CV_8UC1, 255.0 / 95.0); // Using max disparity of 95
        
        // Display raw disparity
        cv::imshow("disparity", normalized);
        
        // Apply color map
        cv::Mat colorized;
        cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
        cv::imshow("disparity_color", colorized);
        
        int key = cv::waitKey(1);
        if(key == 'q') {
            stopPipeline();
        } else if(key == 'j' || key == 'k') {
            // Get current config
            auto config = dai::StereoDepthConfig();
            int threshold = config.getConfidenceThreshold();
            
            // Update threshold
            if(key == 'j') threshold++;
            else if(key == 'k' && threshold > 1) threshold--;
            
            // Set new config
            config.setConfidenceThreshold(threshold);
            std::cout << "Updating threshold to " << threshold << std::endl;
            
            // Send new config
            auto configMsg = std::make_shared<dai::Buffer>();
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

        // Define sources and outputs
        auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::make_pair(400, 400), 30);
        auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::make_pair(400, 400), 30);
        auto depth = pipeline.create<dai::node::StereoDepth>();

        // Configure depth node
        depth->setLeftRightCheck(true);
        depth->setExtendedDisparity(false);
        depth->setSubpixel(false);
        depth->inputConfig.setBlocking(false);

        // Create and configure visualizer node
        auto visualizer = pipeline.create<StereoVisualizer>();
        visualizer->build(depth->disparity);

        // Linking
        monoLeft->requestOutput(std::make_pair(400, 400))->link(depth->left);
        monoRight->requestOutput(std::make_pair(400, 400))->link(depth->right);

        // Start pipeline
        pipeline.start();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 
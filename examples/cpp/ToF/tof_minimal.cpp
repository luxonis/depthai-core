#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

constexpr float FPS = 30.0f;
int main() {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    constexpr float minDepth = 100.0f;
    constexpr float maxDepth = 7000.0f;

    auto profile = dai::ToFConfig::Profile::MID_RANGE;

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, profile, FPS);

    auto depthOutputQueue = tof->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthOutputQueue->get<dai::ImgFrame>();
        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth, minDepth, maxDepth, cv::COLORMAP_JET, true).getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}

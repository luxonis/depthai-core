#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<dai::beta::node::ToFStereoFusion>()->build(left, right);

    auto depthQueue = fusion->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::imshow("Fused depth", dai::utility::colorizeDepthFrame(*depth).getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }
}

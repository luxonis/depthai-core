// ToF + stereo fusion with the STEREO_NETWORK model: one network takes the rectified stereo pair and the ToF reprojected into
// the rectified-left view, and the device composes its disparity with the ToF. The network ships with the RVC4 firmware.
#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<dai::beta::node::ToFStereoFusion>();
    fusion->setModel(dai::beta::node::ToFStereoFusion::Properties::Model::STEREO_NETWORK);
    fusion->build(left, right);

    auto depthQueue = fusion->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::imshow("Fused depth (rectified left)", dai::utility::colorizeDepthFrame(*depth).getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }
    return 0;
}

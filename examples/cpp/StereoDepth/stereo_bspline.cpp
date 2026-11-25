// Stereo depth rectified with the B-spline lens models of the device calibration (RVC4).
// Uses BSPLINE rectification when the calibration has a B-spline lens model for both stereo cameras, otherwise the
// default LEGACY (OpenCV) rectification. B-spline rectification runs at the native resolution of the lens models.
#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    const auto calibration = pipeline.getDefaultDevice()->readCalibration();
    const auto leftLens = calibration.getBsplineCameraCalibration(dai::CameraBoardSocket::CAM_B);
    const auto rightLens = calibration.getBsplineCameraCalibration(dai::CameraBoardSocket::CAM_C);
    const bool useBspline = leftLens && rightLens;
    const std::pair<uint32_t, uint32_t> size =
        useBspline ? std::pair<uint32_t, uint32_t>(leftLens->width, leftLens->height) : std::pair<uint32_t, uint32_t>(640, 400);
    std::cout << "Rectification: " << (useBspline ? "BSPLINE" : "LEGACY (no B-spline lens models in the calibration)") << " " << size.first << "x"
              << size.second << std::endl;

    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>()->build(*left->requestOutput(size), *right->requestOutput(size));
    stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
    if(useBspline) stereo->setRectificationModel(dai::node::StereoDepth::RectificationModel::BSPLINE);
    auto depthQueue = stereo->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth).getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }
    return 0;
}

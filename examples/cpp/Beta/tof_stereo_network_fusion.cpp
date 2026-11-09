// ToF + stereo fusion with the STEREO_NETWORK model: one network takes the rectified stereo pair and the ToF reprojected into
// the rectified-left view, and the device composes its disparity with the ToF.
//
// Usage: tof_stereo_network_fusion <model.dlc> [udo_registration_library] [width height]
// The model is not in the model zoo. Models with a custom operator need its registration library; see the
// NeuralNetwork backend properties udo_package and output_tensors.
#include <cstdlib>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <model.dlc> [udo_registration_library] [width height]" << std::endl;
        return 1;
    }
    const std::string model = argv[1];
    const std::string udoPackage = argc > 2 ? argv[2] : "";
    const uint32_t width = argc > 4 ? static_cast<uint32_t>(std::stoul(argv[3])) : 1280;
    const uint32_t height = argc > 4 ? static_cast<uint32_t>(std::stoul(argv[4])) : 800;

    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<dai::beta::node::ToFStereoFusion>();
    fusion->setModel(dai::beta::node::ToFStereoFusion::Properties::Model::STEREO_NETWORK);
    fusion->setNetworkInputSize(width, height);
    fusion->build(left, right);
    fusion->neuralNetwork->setModelPath(model);
    std::map<std::string, std::string> backendProperties{{"output_tensors", "disp_low,head9,conf"}};
    if(!udoPackage.empty()) backendProperties["udo_package"] = udoPackage;
    fusion->neuralNetwork->setBackendProperties(backendProperties);
    // Keep pixels at depth Z with confidence >= 0.6 * Z / 10 m, and output the ToF depth closer than the network can.
    fusion->initialConfig->setConfidenceThreshold(0.6f);
    fusion->initialConfig->setNearRangePassthrough(true);

    auto depthQueue = fusion->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::imshow("Fused depth (rectified left)", dai::utility::colorizeDepthFrame(*depth).getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }
    return 0;
}

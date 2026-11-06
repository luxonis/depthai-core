#include <argparse/argparse.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

// Stitch CAM_B and CAM_C of one device into a panorama from their calibration

static constexpr float FPS = 30.0f;
static const char* WINDOW_NAME = "CAM_B + CAM_C panorama";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("stitching_panorama");
    program.add_argument("--deviceIp").default_value(std::string("")).help("Device IP address (default: auto-discover)");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& error) {
        std::cerr << error.what() << std::endl;
        std::cerr << program;
        return 1;
    }
    const auto deviceIp = program.get<std::string>("--deviceIp");

    auto device = deviceIp.empty() ? std::make_shared<dai::Device>() : std::make_shared<dai::Device>(dai::DeviceInfo(deviceIp));
    dai::Pipeline pipeline(device);

    std::vector<dai::Node::Output*> outputs;
    for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        auto camera = pipeline.create<dai::node::Camera>()->build(socket, std::nullopt, FPS);
        // Calibrated composition needs undistorted inputs
        outputs.push_back(camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS, true));
    }

    auto stitching = pipeline.create<dai::node::Stitching>()->build(outputs);
    stitching->setMode(dai::node::Stitching::Mode::PANORAMA);
    stitching->setCameraModel(dai::CameraModel::Perspective);
    // Uncomment to trade seam quality for throughput.
    // stitching->setSeamFinder(dai::node::Stitching::SeamFinder::NONE);
    // Uncomment to register the images visually instead of using the calibration.
    // stitching->setUseInputCalibration(false);
    stitching->setMaxPanoramaSize(2000, 1000);
    stitching->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(2.0 / FPS)));
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    while(pipeline.isRunning()) {
        auto message = output->tryGet<dai::ImgFrame>();
        if(message != nullptr) {
            cv::imshow(WINDOW_NAME, message->getCvFrame());
        }
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }
    pipeline.stop();
    cv::destroyAllWindows();
    return 0;
}

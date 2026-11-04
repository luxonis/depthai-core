#include <argparse/argparse.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

// Project CAM_B and CAM_C of one device onto a plane (bird's-eye view)

static constexpr float FPS = 10.0f;
static const char* WINDOW_NAME = "CAM_B + CAM_C planar projection";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("stitching_planar_projection");
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
        outputs.push_back(camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS));
    }

    auto stitching = pipeline.create<dai::node::Stitching>()->build(outputs);
    stitching->setMode(dai::node::Stitching::Mode::PLANAR_PROJECTION);
    stitching->setPlane(dai::Point3f(0, 0, 130),  // A point on the plane, in centimetres
                        dai::Point3f(0, 1, 1),    // Plane normal
                        dai::LengthUnit::CENTIMETER);
    stitching->setMaxRange(2.0f, dai::LengthUnit::METER);
    stitching->setMaxViewSize(1280, 720);
    stitching->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(2.0 / FPS)));
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        cv::imshow(WINDOW_NAME, output->get<dai::ImgFrame>()->getCvFrame());
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }
    pipeline.stop();
    return 0;
}

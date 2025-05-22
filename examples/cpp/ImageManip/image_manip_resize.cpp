#include <iostream>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

int main(int argc, char** argv) {
    std::shared_ptr<dai::Device> device = nullptr;
    if(argc <= 1) {
        device = std::make_shared<dai::Device>();
    } else {
        device = std::make_shared<dai::Device>(argv[1]);
    }
    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    // Resize to 400x400 and avoid stretching by cropping from the center
    manip->initialConfig->setOutputSize(400, 400, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
    // Set output frame type
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::RGB888i);

    camRgb->requestOutput((std::make_pair(1920, 1080)))->link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.run();
}

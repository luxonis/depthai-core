#include <iostream>
#include <memory>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

int main(int argc, char** argv) {
    std::shared_ptr<dai::Device> device = nullptr;
    if(argc <= 1) {
        device = std::make_shared<dai::Device>();
    } else {
        device = std::make_shared<dai::Device>(argv[1]);
    }
    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManipV2>();

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

    manip->setMaxOutputFrameSize(4000000);
    manip->initialConfig.setOutputSize(1280, 720, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
    manip->initialConfig.setBackgroundColor(100, 100, 100);
    manip->initialConfig.rotateDeg(45);
    manip->initialConfig.crop(100, 100, 800, 600);
    manip->initialConfig.flipVertical();
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);

    camRgb->video.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();
    pipeline.wait();
}

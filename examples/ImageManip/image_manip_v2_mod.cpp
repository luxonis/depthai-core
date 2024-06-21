#include <iostream>
#include <memory>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/ImageManipHost.hpp"

using namespace dai::impl;
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

    // First rotate the frame around the center by 45 degrees (clockwise), then scale it down to half its size, then crop a rectangle beginning at (0, 0) with width 400 and height 400, then flip it vertically. At the end resize the frame to 1270x710 and keep the aspect ratio by cropping from the center. Set output frame type to RGB888i.
    manip->initialConfig.setOutputSize(1270, 710, dai::ImageManipConfigV2::ResizeMode::CENTER_CROP);
    manip->initialConfig.rotateDeg(45);
    manip->initialConfig.scale(0.5);
    manip->initialConfig.crop(50, 100, 200, 200);
    manip->initialConfig.flipVertical();
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888i);

    manip->setMaxOutputFrameSize(2709360);

    camRgb->video.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(30));

    pipeline.stop();
}

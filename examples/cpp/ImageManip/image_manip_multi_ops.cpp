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

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    manip->setMaxOutputFrameSize(4000000);
    manip->initialConfig->setOutputSize(1280, 720, dai::ImageManipConfig::ResizeMode::LETTERBOX);
    manip->initialConfig->setBackgroundColor(100, 100, 100);
    manip->initialConfig->addRotateDeg(45);
    manip->initialConfig->addCrop(100, 100, 800, 600);
    manip->initialConfig->addFlipVertical();
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::RGB888p);

    auto* rgbOut = camRgb->requestOutput({1920, 1080});
    rgbOut->link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();
    pipeline.wait();
}
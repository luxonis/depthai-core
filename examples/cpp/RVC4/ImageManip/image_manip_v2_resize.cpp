#include <iostream>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/ImageManipV2.hpp"
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

    // Resize to 400x400 and avoid stretching by cropping from the center
    manip->initialConfig.setOutputSize(400, 400, dai::ImageManipConfigV2::ResizeMode::CENTER_CROP);
    // Set output frame type
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888i);

    camRgb->video.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(30));

    pipeline.stop();
}

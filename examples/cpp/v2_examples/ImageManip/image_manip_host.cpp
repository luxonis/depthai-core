#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/ImageManipHost.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

using namespace dai::impl;
int main(int argc, char** argv) {
    if(argc <= 1) {
        std::cout << "Video parameter is missing" << std::endl;
        std::cout << "Usage: ./image_manip_host video_path" << std::endl;
        return -1;
    }

    dai::Pipeline pipeline(false);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManipHost>();
    // After doing the rest of the operations, resize the frame to 1270x710 and keep the aspect ratio by cropping from the center
    manip->initialConfig.setOutputSize(1270, 710, dai::ImageManipConfigV2::ResizeMode::CENTER_CROP);
    manip->initialConfig.rotateDeg(45);
    manip->initialConfig.crop(0, 0, 400, 400);
    manip->initialConfig.scale(0.5);
    manip->initialConfig.flipVertical();
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888i);

    replay->setReplayVideoFile("vid.mp4");
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    replay->setFps(30);

    replay->out.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(30));

    pipeline.stop();
}

#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

int main(int argc, char** argv) {
    if(argc <= 1) {
        std::cout << "Video parameter is missing" << std::endl;
        std::cout << "Usage: ./image_manip_host video_path" << std::endl;
        return -1;
    }

    dai::Pipeline pipeline(false);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManipV2>();
    manip->setRunOnHost();
    // After doing the rest of the operations, resize the frame to 1270x710 and keep the aspect ratio by cropping from the center
    manip->setMaxOutputFrameSize(4000000);
    manip->initialConfig.setOutputSize(1280, 720, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
    manip->initialConfig.setBackgroundColor(100, 100, 100);
    manip->initialConfig.addRotateDeg(45);
    manip->initialConfig.addCrop(100, 100, 800, 600);
    manip->initialConfig.addFlipVertical();
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);

    replay->setReplayVideoFile(argv[1]);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    replay->setFps(30);
    replay->setSize(1280, 720);

    replay->out.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();

    pipeline.wait();
}

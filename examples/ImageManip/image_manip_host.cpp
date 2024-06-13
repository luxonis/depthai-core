#include <iostream>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
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

    dai::Pipeline pipeline;

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto display = pipeline.create<dai::node::Display>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig.setOutputSize(900, 402, dai::ImageManipBase::ResizeMode::CENTER_CROP);
    /*manip->initialConfig.rotateDeg(45);*/
    manip->initialConfig.crop(0, 0, 400, 400);
    manip->initialConfig.flipVertical();

    manip->setMaxOutputFrameSize(8085400);

    replay->setReplayVideoFile(argv[1]);
    replay->setOutFrameType(dai::ImgFrame::Type::RGB888i);
    replay->setFps(30);

    replay->out.link(manip->inputImage);
    manip->out.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(20));

    pipeline.stop();
}

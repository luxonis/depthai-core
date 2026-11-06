#include <iostream>

#include "depthai/depthai.hpp"

// Replay into a processing node on the host. No device is required.
// ImageManip stands in for the processing step. A NeuralNetwork can be linked the same way.
int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: replay_to_processing <video-or-directory>\n";
        return 1;
    }

    dai::Pipeline pipeline(false);
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(argv[1]);
    replay->setRunOnHost(true);
    replay->setOutFrameType(dai::ImgFrame::Type::NV12);
    replay->setFps(30);
    replay->setLoop(false);
    replay->setStopPipelineAtEnd(true);

    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->setRunOnHost();
    manip->setMaxOutputFrameSize(4 * 1024 * 1024);
    manip->initialConfig->setOutputSize(320, 240, dai::ImageManipConfig::ResizeMode::LETTERBOX);
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::RGB888p);

    replay->out.link(manip->inputImage);
    auto queue = manip->out.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto frame = queue->tryGet<dai::ImgFrame>();
        if(frame == nullptr) continue;
        std::cout << frame->getWidth() << "x" << frame->getHeight() << " sequence " << frame->getSequenceNum() << "\n";
    }
    pipeline.stop();
    return 0;
}

#include <argparse/argparse.hpp>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"

// Replay into a host ImageManip node. No device is required.
int main(int argc, char** argv) {
    argparse::ArgumentParser program("replay_to_processing", "1.0", argparse::default_arguments::help);
    program.add_argument("video").help("Video file or frame directory");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    try {
        dai::Pipeline pipeline(false);
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(program.get<std::string>("video"));
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
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}

#include <argparse/argparse.hpp>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"

// Replay into a camera node. Requires a connected RVC4 device.
int main(int argc, char** argv) {
    argparse::ArgumentParser program("replay_to_camera", "1.0", argparse::default_arguments::help);
    program.add_argument("video").help("Video file or frame directory");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    try {
        dai::Pipeline pipeline;
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(program.get<std::string>("video"));
        replay->setFps(30);
        replay->setLoop(true);

        auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, *replay);
        auto* output = camera->requestOutput(std::make_pair<std::uint32_t, std::uint32_t>(640, 480));
        auto queue = output->createOutputQueue();

        pipeline.start();
        while(pipeline.isRunning()) {
            auto frame = queue->get<dai::ImgFrame>();
            if(frame == nullptr) break;
            std::cout << frame->getWidth() << "x" << frame->getHeight() << " sequence " << frame->getSequenceNum() << "\n";
            if(frame->getSequenceNum() >= 30) break;
        }
        pipeline.stop();
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}

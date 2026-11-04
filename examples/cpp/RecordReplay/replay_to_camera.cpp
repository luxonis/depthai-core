#include <iostream>

#include "depthai/depthai.hpp"

// Replay into a camera node. The camera outputs the frames as if they came from the sensor.
// This needs a connected RVC4 device. ReplayVideo runs on the device and feeds mockIsp.
int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: replay_to_camera <video-or-directory>\n";
        return 1;
    }

    dai::Pipeline pipeline;
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(argv[1]);
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
    return 0;
}

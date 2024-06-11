#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

int main() {
    dai::Pipeline pipeline(true);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto cam = pipeline.create<dai::node::ColorCamera>()->build();
    auto display = pipeline.create<dai::node::Display>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setMockIspSize(1280, 720);

    replay->setReplayVideoFile("video2.mp4");
    replay->setFps(30);
    replay->setOutFrameType(dai::ImgFrame::Type::YUV420p);

    replay->out.link(cam->mockIsp);
    cam->video.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(20));

    pipeline.stop();
}

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

int main(int argc, char** argv) {
    std::string vidName = "video";
    if(argc > 1) vidName = argv[1];
    dai::Pipeline pipeline(true);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<dai::node::Display>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    replay->setReplayVideoFile(vidName + ".mp4");
    replay->setReplayMetadataFile(vidName + ".mcap");
    replay->setOutFrameType(dai::ImgFrame::Type::YUV420p);
    replay->setSize(1920, 1080);
    /*replay->setFps(30);*/

    replay->out.link(cam->mockIsp);
    cam->video.link(display->input);

    pipeline.run();  // Let the display node stop the pipeline
}

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

int main() {
    dai::Pipeline pipeline(true);

    auto replay = pipeline.create<dai::node::Replay>();
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<dai::node::Display>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    // Check if the files exist
    std::ifstream file("recording.mp4");
    if(!file.good()) throw std::runtime_error("File 'recording.mp4' not found - first run 'record_encoded_h264' example");
    file = std::ifstream("recording.mcap");
    if(!file.good()) throw std::runtime_error("File 'recording.mcap' not found - first run 'record_encoded_h264' example");
    replay->setReplayVideo("recording.mp4");
    replay->setReplayFile("recording.mcap");
    replay->setFps(10);
    replay->setOutFrameType(dai::ImgFrame::Type::YUV420p);

    replay->out.link(cam->mockIsp);
    cam->video.link(display->input);

    pipeline.run(); // Let the display node stop the pipeline
}

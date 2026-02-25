#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

int main(int argc, char** argv) {
    std::string vidName = "test_video.avi";
    if(argc > 1) vidName = argv[1];
    dai::Pipeline pipeline(false);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto display = pipeline.create<dai::node::Display>();

    replay->setReplayVideoFile(vidName);
    replay->setFps(30);

    replay->out.link(display->input);

    pipeline.run();  // Let the display node stop the pipeline
}

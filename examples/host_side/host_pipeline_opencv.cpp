#include <iostream>

// project
#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto display = pipeline.create<dai::node::Display>();
    camRgb->preview.link(display->input);
    pipeline.start();
    pipeline.wait();
    return 0;
}

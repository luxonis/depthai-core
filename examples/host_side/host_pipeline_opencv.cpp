#include <iostream>

// project
#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/HostCamera.hpp"
int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    auto hostCam = pipeline.create<dai::node::HostCamera>();
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto displayDevice = pipeline.create<dai::node::Display>(std::string{"Device Display"});
    auto displayHost = pipeline.create<dai::node::Display>("Host Display");
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    camRgb->preview.link(displayDevice->input);
    hostCam->out.link(imageManip->inputImage);
    imageManip->out.link(displayHost->input);
    pipeline.start();
    pipeline.wait();
    return 0;
}

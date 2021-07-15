
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline p;

    // Define a source - color camera
    auto colorCam = p.create<dai::node::ColorCamera>();

    // Script node
    auto script = p.create<dai::node::Script>();
    script->setScriptData(R"(
        import time
        ctrl = CameraControl()
        ctrl.setCaptureStill(True)
        while True:
            time.sleep(1)
            node.io['out'].send(ctrl)
    )");

    // XLinkOut
    auto xout = p.create<dai::node::XLinkOut>();
    xout->setStreamName("still");

    // Connections
    script->outputs["out"].link(colorCam->inputControl);
    colorCam->still.link(xout->input);

    return p;
}

int main() {
    using namespace std;

    dai::Pipeline p = createPipeline();
    dai::Device d(p);

    auto still = d.getOutputQueue("still");

    while(1) {
        auto img = still->get<dai::ImgFrame>();
        cv::imshow("still", img->getCvFrame());

        if(cv::waitKey(1) == 'q') {
            return 0;
        }
    }

    return 0;
}

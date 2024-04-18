#include <iostream>

// project
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/HostNode.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/HostCamera.hpp"
#include "depthai/properties/ColorCameraProperties.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);
    auto camRgb = pipeline.create<dai::node::ColorCamera>(dai::CameraBoardSocket::CAM_A);
    camRgb->setVideoSize(640, 480);
    auto displayDevice = pipeline.create<dai::node::Display>(std::string{"Device Display"});

    // camRgb->video.link(displayDevice->input);

    // Option 2:
    auto queue = camRgb->video.createQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto message = queue->get<dai::ImgFrame>();
        cv::imshow("QueueFrame", message->getCvFrame());
        auto q = cv::waitKey(1);
        if(q == 'q') {
            pipeline.stop();
            break;
        }
    }

    pipeline.wait();
    return 0;
}

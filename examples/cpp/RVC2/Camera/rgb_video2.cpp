// Includes common necessary includes for development using depthai library
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    auto outputQueue = camRgb->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12)->createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto videoIn = outputQueue->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
        }
    }
    return 0;
}

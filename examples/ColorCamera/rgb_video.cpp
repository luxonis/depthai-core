// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    camRgb->setVideoSize(1920, 1080);
    // camRgb->setVideoSize(1280, 720);

    auto outputQueue = camRgb->video.createQueue();

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

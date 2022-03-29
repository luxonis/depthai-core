#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Pass the argument `uvc` to run in UVC mode

int main(int argc, char** argv) {
    bool enableUVC = 0;
    if(argc > 1) {
        if (std::string(argv[1]) == "uvc") {
            enableUVC = 1;
        } else {
            printf("Unrecognized argument: %s\n", argv[1]);
        }
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
//    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->setIspScale(1, 2);
    camRgb->setVideoSize(1920, 1080);
    camRgb->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_60_HZ);
    camRgb->setFps(30);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    if (enableUVC) {
        auto uvc = pipeline.create<dai::node::UVC>();
        camRgb->video.link(uvc->input);
        //camRgb->video.link(xoutVideo->input); // Could actually keep this as well
    } else {
        camRgb->video.link(xoutVideo->input);
    }

    // Connect to device and start pipeline
    auto config = dai::Device::Config();
    config.board.uvcEnable = enableUVC;
    printf("=== Creating device with board config...\n");
    dai::Device device(config);
    printf("=== Device created, starting pipeline...\n");
    device.startPipeline(pipeline);
    printf("=== Started!\n");
    if (enableUVC) printf(">>> Keep this running, and open a separate UVC viewer\n");

    int qsize = 1;
    bool blocking = false;
    auto video = device.getOutputQueue("video", qsize, blocking);

    using namespace std::chrono;
    auto tprev = steady_clock::now();
    int count = 0;

    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();

        if (1) { // FPS calc
            auto tnow = steady_clock::now();
            count++;
            auto tdiff = duration<double>(tnow - tprev).count();
            if (tdiff >= 1) {
                double fps = count / tdiff;
                printf("FPS: %.3f\n", fps);
                count = 0;
                tprev = tnow;
            }
        }

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}

// Device-to-device link in ONE dai::Pipeline.
//
// A Camera on device A is linked directly to an ImageManip running on device B.
// The pipeline inserts the host relay automatically at build time (visible as one
// info log line); no manual forwarding is needed.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

static std::atomic_bool running{true};

int main(int argc, char** argv) {
    signal(SIGINT, [](int) { running = false; });

    auto deviceInfos = dai::Device::getAllAvailableDevices();
    if(argc >= 3) {
        deviceInfos = {dai::DeviceInfo(argv[1]), dai::DeviceInfo(argv[2])};
    }
    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        return 0;
    }

    dai::Pipeline pipeline(false);
    auto deviceA = pipeline.addDevice(deviceInfos[0]);
    auto deviceB = pipeline.addDevice(deviceInfos[1]);

    auto camera = pipeline.create<dai::node::Camera>(deviceA)->build(dai::CameraBoardSocket::CAM_A);
    auto manip = pipeline.create<dai::node::ImageManip>(deviceB);
    manip->initialConfig->setOutputSize(320, 200);

    // Cross-device link: relayed through the host automatically
    camera->requestOutput(std::make_pair(640, 400))->link(manip->inputImage);

    auto queue = manip->out.createOutputQueue();
    pipeline.start();

    std::cout << "Camera on " << deviceA->getDeviceId() << " -> ImageManip on " << deviceB->getDeviceId() << std::endl;

    const char* displayEnv = std::getenv("DISPLAY");
    const bool display = displayEnv != nullptr && displayEnv[0] != '\0';
    while(running && pipeline.isRunning()) {
        bool hasTimedOut = false;
        auto frame = queue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);
        if(frame == nullptr) continue;
        std::cout << "Frame processed on device B: " << frame->getWidth() << "x" << frame->getHeight() << std::endl;
        if(display) {
            cv::imshow("device_to_device_relay", frame->getCvFrame());
            if(cv::waitKey(1) == 'q') break;
        }
    }

    pipeline.stop();
    return 0;
}

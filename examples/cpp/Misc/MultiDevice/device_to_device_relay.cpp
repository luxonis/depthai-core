
// Device-to-device link in ONE dai::Pipeline.
//
// A Camera on device A is linked directly to an ImageManip running on device B.
// At build time the pipeline sends the frames directly from A to B when both are
// RVC4 devices that reach each other, and relays them through the host otherwise.
// One info log line per link tells which it is, and why; no manual forwarding is
// needed. Pass --relay to always relay through the host, to compare.
//
// Usage: device_to_device_relay [deviceA deviceB] [--relay]
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

static std::atomic_bool running{true};

int main(int argc, char** argv) {
    signal(SIGINT, [](int) { running = false; });

    bool forceRelay = false;
    std::vector<std::string> args;
    for(int i = 1; i < argc; i++) {
        if(std::string(argv[i]) == "--relay") {
            forceRelay = true;
        } else {
            args.emplace_back(argv[i]);
        }
    }
    auto deviceInfos = dai::Device::getAllAvailableDevices();
    if(args.size() >= 2) {
        deviceInfos = {dai::DeviceInfo(args[0]), dai::DeviceInfo(args[1])};
    }
    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        return 0;
    }

    dai::Pipeline pipeline(false);
    auto deviceA = pipeline.addDevice(deviceInfos[0]);
    auto deviceB = pipeline.addDevice(deviceInfos[1]);
    if(forceRelay) {
        pipeline.setCrossDeviceLinkMode(dai::CrossDeviceLinkMode::HOST_RELAY);
    }

    auto camera = pipeline.create<dai::node::Camera>(deviceA)->build(dai::CameraBoardSocket::CAM_A);
    auto manip = pipeline.create<dai::node::ImageManip>(deviceB);
    manip->initialConfig->setOutputSize(320, 200);

    // Cross-device link: direct between the devices, or relayed through the host
    camera->requestOutput(std::make_pair(640, 400))->link(manip->inputImage);

    auto queue = manip->out.createOutputQueue();
    pipeline.start();

    while(running && pipeline.isRunning()) {
        bool hasTimedOut = false;
        auto frame = queue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);
        if(frame == nullptr) continue;
        cv::imshow("device_to_device_relay", frame->getCvFrame());
        if(cv::waitKey(1) == 'q') break;
    }

    pipeline.stop();
    return 0;
}

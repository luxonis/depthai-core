#include <algorithm>
#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

std::atomic<bool> gQuit(false);

void signalHandler(int) {
    gQuit = true;
}

void printDrivers(std::shared_ptr<dai::Device>& device) {
    std::cout << "Detected IR drivers:\n";
    for(const auto& d : device->getIrDrivers()) {
        std::cout << "  name=" << std::get<0>(d)
                  << "  bus=" << std::get<1>(d)
                  << "  addr=0x" << std::hex << std::get<2>(d) << std::dec << "\n";
    }
}

void applyLeds(std::shared_ptr<dai::Device>& device, float dot, float flood, int mask) {
    bool dotOk = device->setIrLaserDotProjectorIntensity(dot, mask);
    bool floodOk = device->setIrFloodLightIntensity(flood, mask);
    std::cout << "dot=" << dot << "  flood=" << flood
              << "  mask=0x" << std::hex << mask << std::dec
              << "  dot_ok=" << dotOk << "  flood_ok=" << floodOk << "\n";
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        auto device = std::make_shared<dai::Device>();
        std::cout << "Connected to: " << device->getDeviceName() << "\n";
        printDrivers(device);

        dai::Pipeline pipeline(device);

        auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        auto leftOut = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
        auto rightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

        auto leftQueue = leftOut->createOutputQueue();
        auto rightQueue = rightOut->createOutputQueue();

        float dotIntensity = 0.0f;
        float floodIntensity = 0.0f;
        int mask = -1;  // all drivers/sides

        applyLeds(device, dotIntensity, floodIntensity, mask);

        pipeline.start();

        std::cout << "Controls:\n"
                  << "  W/S : increase/decrease dot projector intensity\n"
                  << "  A/D : increase/decrease flood light intensity\n"
                  << "  1   : apply only to LEFT (mask 0x1)\n"
                  << "  2   : apply only to RIGHT (mask 0x2)\n"
                  << "  3   : apply to ALL (mask -1)\n"
                  << "  P   : re-print IR drivers\n"
                  << "  Q   : quit\n";

        while(pipeline.isRunning() && !gQuit) {
            auto left = leftQueue->get<dai::ImgFrame>();
            auto right = rightQueue->get<dai::ImgFrame>();
            if(!left || !right) continue;

            cv::imshow("left", left->getCvFrame());
            cv::imshow("right", right->getCvFrame());

            int key = cv::waitKey(1);
            bool changed = false;

            if(key == 'q' || key == 'Q' || key == 27) {
                break;
            } else if(key == 'w' || key == 'W') {
                dotIntensity = std::min(dotIntensity + 0.1f, 1.0f);
                changed = true;
            } else if(key == 's' || key == 'S') {
                dotIntensity = std::max(dotIntensity - 0.1f, 0.0f);
                changed = true;
            } else if(key == 'a' || key == 'A') {
                floodIntensity = std::min(floodIntensity + 0.1f, 1.0f);
                changed = true;
            } else if(key == 'd' || key == 'D') {
                floodIntensity = std::max(floodIntensity - 0.1f, 0.0f);
                changed = true;
            } else if(key == '1') {
                mask = 0x1;
                changed = true;
            } else if(key == '2') {
                mask = 0x2;
                changed = true;
            } else if(key == '3') {
                mask = -1;
                changed = true;
            } else if(key == 'p' || key == 'P') {
                printDrivers(device);
            }

            if(changed) {
                applyLeds(device, dotIntensity, floodIntensity, mask);
            }
        }

        pipeline.stop();
        pipeline.wait();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

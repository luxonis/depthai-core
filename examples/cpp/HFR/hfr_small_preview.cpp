#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"

constexpr std::pair<int, int> SIZE = {1280, 720};
constexpr int FPS = 480;

int main() {
    dai::Pipeline pipeline;

    auto device = pipeline.getDefaultDevice();
    auto platform = device->getPlatform();
    if(platform != dai::Platform::RVC4) {
        std::cerr << "This example is only supported on IMX586 and Luxonis OS 1.20.5 or higher\n" << std::flush;
        return 0;
    }

    // Exit cleanly if the selected HFR mode is not advertised by CAM_A.
    bool supportsRequestedFps = false;
    for(const auto& cameraFeature : device->getConnectedCameraFeatures()) {
        if(cameraFeature.socket != dai::CameraBoardSocket::CAM_A) continue;
        for(const auto& config : cameraFeature.configs) {
            if(config.width == SIZE.first && config.height == SIZE.second && config.maxFps >= static_cast<float>(FPS)) {
                supportsRequestedFps = true;
                break;
            }
        }
        break;
    }
    if(!supportsRequestedFps) {
        std::cerr << "This example is only supported on IMX586 and Luxonis OS 1.20.5 or higher\n" << std::flush;
        return 0;
    }

    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::AUTO, std::nullopt, static_cast<float>(FPS));
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(FPS);

    auto* output = cam->requestOutput(std::make_pair(250U, 250U));
    output->link(benchmarkIn->input);

    auto outputQueue = output->createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto imgFrame = outputQueue->get<dai::ImgFrame>();
        if(imgFrame == nullptr) continue;
        cv::imshow("frame", imgFrame->getCvFrame());
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}

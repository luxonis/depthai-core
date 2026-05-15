#include <iostream>
#include <optional>

#include "depthai/depthai.hpp"

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
            if(config.width == 1280 && config.height == 720 && config.maxFps >= static_cast<float>(FPS)) {
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

    auto cameraNode = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::nullopt, static_cast<float>(FPS));

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(cameraNode, "yolov6-nano");

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(FPS);
    detectionNetwork->out.link(benchmarkIn->input);

    auto qDet = detectionNetwork->out.createOutputQueue();
    pipeline.start();

    while(pipeline.isRunning()) {
        auto inDet = qDet->get<dai::ImgDetections>();
        if(inDet == nullptr) continue;
        (void)inDet;
    }

    return 0;
}

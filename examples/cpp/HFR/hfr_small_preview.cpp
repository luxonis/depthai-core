#include <opencv2/opencv.hpp>
#include <iostream>
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

    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(FPS);

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->initialConfig->setOutputSize(250, 250);
    imageManip->setMaxOutputFrameSize(static_cast<int>(250 * 250 * 1.6));

    // One of the two modes can be selected
    // NOTE: Generic resolutions are not yet supported through camera node when using HFR mode
    auto* output = cam->requestOutput(SIZE, std::nullopt, dai::ImgResizeMode::CROP, static_cast<float>(FPS));

    output->link(imageManip->inputImage);
    imageManip->out.link(benchmarkIn->input);

    auto outputQueue = imageManip->out.createOutputQueue();

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

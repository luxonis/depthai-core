#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    const std::string directory = argc > 1 ? argv[1] : ".";
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto record = pipeline.create<dai::node::RecordVideo>();
    record->setFormat(dai::RecordFormat::RAW_FRAMES);
    record->setOutputDirectory(directory);
    record->setStreamName("frames");

    auto* cameraOut = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    cameraOut->link(record->input);
    auto status = record->status.createOutputQueue(16, true);

    pipeline.start();
    std::cout << "Recording raw frames. Press Ctrl+C to stop.\n";
    while(pipeline.isRunning()) {
        auto message = status->tryGet<dai::RecordStatus>();
        if(message && message->state == dai::RecordStatus::State::STARTED) {
            std::cout << "Recording " << message->path << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}

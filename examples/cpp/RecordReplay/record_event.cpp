#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/RecordControl.hpp"
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
    record->setStreamName("event");
    record->setWaitForTrigger(true);
    record->setPreTriggerFrames(30);

    auto* cameraOut = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
    cameraOut->link(record->input);

    // RecordControl is built on the host. A Script node on the device cannot construct this
    // message until the device firmware accepts RecordControl.
    auto control = record->inputControl.createInputQueue();
    auto status = record->status.createOutputQueue(16, true);

    pipeline.start();
    std::cout << "Waiting. The last 30 frames are kept. Press Enter to record them and what follows.\n";
    std::cin.get();
    control->send(dai::RecordControl::start());
    std::cout << "Recording. Press Enter to stop.\n";
    std::cin.get();
    control->send(dai::RecordControl::stop());

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(std::chrono::steady_clock::now() < deadline) {
        auto message = status->tryGet<dai::RecordStatus>();
        if(message && message->state == dai::RecordStatus::State::STOPPED) {
            std::cout << "Stopped segment " << message->segment << " with " << message->frameCount << " frames\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    pipeline.stop();
    return 0;
}

#include <chrono>
#include <iostream>

// DepthAI and OpenCV headers
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/Gate.hpp"

int main() {
    dai::Pipeline pipeline;

    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto cameraOut = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, 30);

    auto gate = pipeline.create<dai::node::Gate>();

    cameraOut->link(gate->input);

    auto cameraQueue = gate->output.createOutputQueue();

    auto gateControlQueue = gate->inputControl.createInputQueue();

    pipeline.start();

    auto startTime = std::chrono::steady_clock::now();
    bool openGate = true;

    while(pipeline.isRunning()) {
        auto frame = cameraQueue->tryGet<dai::ImgFrame>();
        if(frame != nullptr) {
            cv::imshow("camera", frame->getCvFrame());
        }

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;

        if(elapsed.count() > 5.0) {
            openGate = !openGate;

            if(openGate) {
                gateControlQueue->send(dai::GateControl::openGate());
            } else {
                gateControlQueue->send(dai::GateControl::closeGate());
            }

            std::cout << "Gate toggled to: " << (openGate ? "True" : "False") << std::endl;

            startTime = currentTime;
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}

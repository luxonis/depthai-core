#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

/*
    High conversion gain (HCG) camera example
    Use 'h' to toggle high conversion gain on/off
    Use 'q' to quit

    HCG raises the sensor's conversion gain (about 2.6x on IMX462): less noise in low light,
    but pixels saturate about 2.6x sooner. Auto exposure compensates for the extra signal,
    so watch the exposure and ISO shown on the frame drop when HCG is enabled.

    This example only works on IMX462 sensors.
*/

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto cameraFeatures = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    bool success = false;
    for(const auto& feature : cameraFeatures) {
        if(feature.socket == dai::CameraBoardSocket::CAM_A && feature.sensorName == "IMX462") {
            success = true;
            break;
        }
    }

    if(!success) {
        throw std::runtime_error("No suitable camera found, high conversion gain is only supported on IMX462 sensors!");
    }

    bool hcg = false;
    cam->initialControl.setMisc("high-conversion-gain", static_cast<int>(hcg));
    auto cameraControlQueue = cam->inputControl.createInputQueue();
    auto videoQueue = cam->requestOutput({1280, 720})->createOutputQueue();

    // Start pipeline
    pipeline.start();
    while(pipeline.isRunning()) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;

        cv::Mat frame = videoIn->getCvFrame();
        auto exposureMs = std::chrono::duration<double, std::milli>(videoIn->getExposureTime()).count();
        std::ostringstream text;
        text << "HCG " << (hcg ? "on" : "off") << ", exposure " << std::fixed << std::setprecision(2) << exposureMs << " ms, ISO "
             << videoIn->getSensitivity();
        cv::putText(frame, text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::imshow("video", frame);
        int key = cv::waitKey(1);

        if(key == 'h') {
            hcg = !hcg;
            auto ctrl = std::make_shared<dai::CameraControl>();
            ctrl->setMisc("high-conversion-gain", static_cast<int>(hcg));
            cameraControlQueue->send(ctrl);
            std::cout << "High conversion gain " << (hcg ? "enabled" : "disabled") << std::endl;
        }
        if(key == 'q') {
            break;
        }
    }

    return 0;
}

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

/*
    HDR Camera example
    Use 'h' and 'j' keys to increase/decrease HDR exposure ratio
    Use 'w' and 'e' keys to increase/decrease local tone weight

    This example only works on IMX586/582 sensors.
*/

float clamp(float n, float smallest, float largest) {
    return std::max(smallest, std::min(n, largest));
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto cameraFeatures = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    bool success = false;
    for(const auto& feature : cameraFeatures) {
        if(feature.socket == dai::CameraBoardSocket::CAM_A) {
            if(feature.sensorName == "IMX586" || feature.sensorName == "IMX582" || feature.sensorName == "LCM48") {
                success = true;
                break;
            }
        }
    }

    if(!success) {
        throw std::runtime_error("No suitable camera found, HDR is only supported on IMX586/582 sensors!");
    }
    cam->initialControl.setHdr(true);
    auto cameraControlQueue = cam->inputControl.createInputQueue();
    auto videoQueue = cam->requestOutput({1280, 800})->createOutputQueue();

    // Initialize variables
    float minRatio = 1.0f;
    float maxRatio = 8.0f;
    float ratio = 4.0f;

    float minWeight = 0.0f;
    float maxWeight = 1.0f;
    float weight = 0.75f;

    std::string base = "long";

    // Start pipeline
    pipeline.start();
    while(pipeline.isRunning()) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;

        cv::imshow("video", videoIn->getCvFrame());
        int key = cv::waitKey(1);
        auto ctrl = std::make_shared<dai::CameraControl>();

        // Key handling
        if(key == 'h') {
            ratio = clamp(ratio * 2.0f, minRatio, maxRatio);
            ctrl->setMisc("hdr-exposure-ratio", ratio);
            std::cout << "Setting HDR exposure ratio to " << ratio << std::endl;
            cameraControlQueue->send(ctrl);
        } else if(key == 'j') {
            ratio = clamp(ratio / 2.0f, minRatio, maxRatio);
            ctrl->setMisc("hdr-exposure-ratio", ratio);
            std::cout << "Setting HDR exposure ratio to " << ratio << std::endl;
            cameraControlQueue->send(ctrl);
        } else if(key == 'w') {
            weight = clamp(weight + 1/32.0f, minWeight, maxWeight);
            ctrl->setMisc("hdr-local-tone-weight", weight);
            std::cout << "Setting HDR local tone weight to " << weight << std::endl;
            cameraControlQueue->send(ctrl);
        } else if(key == 'e') {
            weight = clamp(weight - 1/32.0f, minWeight, maxWeight);
            ctrl->setMisc("hdr-local-tone-weight", weight);
            std::cout << "Setting HDR local tone weight to " << weight << std::endl;
            cameraControlQueue->send(ctrl);
        } else if(key == 'b') {
            base = (base == "long") ? "middle" : "long";
            ctrl->setMisc("hdr-exposure-base", base);
            std::cout << "Setting HDR exposure base to '" << base << "'" << std::endl;
            cameraControlQueue->send(ctrl);
        } else if(key == 'q') {
            break;
        }
    }

    return 0;
}
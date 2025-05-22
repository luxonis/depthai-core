#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "depthai/depthai.hpp"

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Map to store output queues
    std::map<std::string, std::shared_ptr<dai::MessageQueue>> outputQueues;

    // Get connected cameras
    auto sockets = device->getConnectedCameras();
    for(const auto& socket : sockets) {
        auto cam = pipeline.create<dai::node::Camera>();
        cam->build(socket);
        auto output = cam->requestFullResolutionOutput();
        outputQueues[dai::toString(socket)] = output->createOutputQueue();
    }

    pipeline.start();
    while(true) {
        for(const auto& [name, queue] : outputQueues) {
            auto videoIn = queue->get<dai::ImgFrame>();
            if(videoIn != nullptr) {
                // Visualizing the frame on slower hosts might have overhead
                cv::imshow(name, videoIn->getCvFrame());
            }
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
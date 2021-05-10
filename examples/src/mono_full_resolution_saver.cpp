#include <chrono>
#include <cstdio>
#include <iostream>

#include "errno.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutRight->setStreamName("right");

    // Properties
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    // Linking
    monoRight->out.link(xoutRight->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queue
    auto qRight = device.getOutputQueue("right", 4, false);

    std::string dirName = "mono_data";
    createDirectory(dirName);

    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        // Frame is transformed and ready to be shown
        cv::imshow("right", inRight->getCvFrame());

        uint64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::stringstream videoStr;
        videoStr << dirName << "/" << time << ".png";
        // After showing the frame, it's being stored inside a target directory as a PNG image
        cv::imwrite(videoStr.str(), inRight->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}

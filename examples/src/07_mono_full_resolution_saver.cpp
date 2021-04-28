#include <cstdio>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv)
{
    using namespace std;
    using namespace std::chrono;

    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutRight->setStreamName("right");

    // Properties
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    // Linking
    camRight->out.link(xoutRight->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device device(pipeline);
    // Start pipeline
    device.startPipeline();

    // Queue
    auto qRight = device.getOutputQueue("right", 4, false);

    mkdir("07_data", 0777);

    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        // Frame is transformed and ready to be shown
        cv::imshow("right", inRight->getCvFrame());

        uint64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::stringstream videoStr;
        videoStr << "07_data/" << time << ".png";
        // After showing the frame, it's being stored inside a target directory as a PNG image
        cv::imwrite(videoStr.str(), inRight->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q')
            return 0;
    }
    return 0;
}

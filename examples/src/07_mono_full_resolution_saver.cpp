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

    dai::Pipeline p;

    // Define a source - color camera
    auto camRight = p.create<dai::node::MonoCamera>();
    auto xoutRight = p.create<dai::node::XLinkOut>();

    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    xoutRight->setStreamName("right");

    // Create outputs
    camRight->out.link(xoutRight->input);


    // Pipeline is defined, now we can connect to the device
    dai::Device d(p);
    // Start pipeline
    d.startPipeline();

    // Queue
    auto q = d.getOutputQueue("right", 4, false);

    mkdir("07_data", 0777);

    while(true) {
        auto inRight = q->get<dai::ImgFrame>();
        cv::Mat frameRight = inRight->getCvFrame();
        // Frame is transformed and ready to be shown
        cv::imshow("right", frameRight);

        uint64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::stringstream videoStr;
        videoStr << "07_data/" << time << ".png";
        // After showing the frame, it's being stored inside a target directory as a PNG image
        cv::imwrite(videoStr.str(), frameRight);

        int key = cv::waitKey(1);
        if(key == 'q')
            return 0;
    }

    return 0;
}

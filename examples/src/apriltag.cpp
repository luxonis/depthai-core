#include <chrono>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto aprilTag = pipeline.create<dai::node::AprilTag>();

    auto xoutMono = pipeline.create<dai::node::XLinkOut>();
    auto xoutAprilTag = pipeline.create<dai::node::XLinkOut>();

    xoutMono->setStreamName("mono");
    xoutAprilTag->setStreamName("aprilTagData");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);

    aprilTag->initialConfig.setType(dai::AprilTagConfigData::AprilTagType::TAG_36H11);

    // Linking
    aprilTag->passthroughInputImage.link(xoutMono->input);
    monoLeft->out.link(aprilTag->inputImage);

    aprilTag->out.link(xoutAprilTag->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the mono frames from the outputs defined above
    auto monoQueue = device.getOutputQueue("mono", 8, false);
    auto aprilTagQueue = device.getOutputQueue("aprilTagData", 8, false);

    auto color = cv::Scalar(0, 255, 0);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        auto inFrame = monoQueue->get<dai::ImgFrame>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat frame = inFrame->getCvFrame();

        auto aprilTagData = aprilTagQueue->get<dai::AprilTagData>()->getAprilTag();
        for(auto aprilTag : aprilTagData) {
            auto xmin = (int)aprilTag.p.x;
            auto ymin = (int)aprilTag.p.y;
            auto xmax = xmin + (int)aprilTag.p.width;
            auto ymax = ymin + (int)aprilTag.p.height;

            std::stringstream idStr;
            idStr << "ID: " << aprilTag.id;
            cv::putText(frame, idStr.str(), cv::Point(xmin + 10, ymin + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << "fps:" << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, inFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("mono", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}

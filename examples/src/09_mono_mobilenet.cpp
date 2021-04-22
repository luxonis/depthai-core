#include <cstdio>
#include <iostream>
#include <chrono>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;

    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline p;

    // Define source
    auto camRight = p.create<dai::node::MonoCamera>();
    auto manip = p.create<dai::node::ImageManip>();
    auto manipOut = p.create<dai::node::XLinkOut>();
    auto detectionNetwork = p.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = p.create<dai::node::XLinkOut>();

    // Properties
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    detectionNetwork->setConfidenceThreshold(0.5);
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setNumInferenceThreads(2);
    manip->initialConfig.setResize(300, 300);
    manip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);
    manipOut->setStreamName("right");
    nnOut->setStreamName("detections");

    // Create outputs
    camRight->out.link(manip->inputImage);
    manip->out.link(detectionNetwork->input);
    manip->out.link(manipOut->input);
    detectionNetwork->out.link(nnOut->input);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    auto qRight = d.getOutputQueue("right", 4, false);
    auto detections = d.getOutputQueue("detections", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        auto det = detections->get<dai::ImgDetections>();
        cv::Mat frame = inRight->getCvFrame();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto color = cv::Scalar(255, 255, 255);
        auto dets = det->detections;
        for(const auto& d : dets) {
            int x1 = d.xmin * frame.cols;
            int y1 = d.ymin * frame.rows;
            int x2 = d.xmax * frame.cols;
            int y2 = d.ymax * frame.rows;

            int labelIndex = d.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, inRight->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("right", frame);

        int key = cv::waitKey(1);
        if(key == 'q')
            return 0;
    }

    return 0;
}

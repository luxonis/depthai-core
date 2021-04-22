#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

int main(int argc, char** argv) {
    using namespace std;
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
    dai::Pipeline pipeline;

    // Define source
    auto camRight = pipeline.create<dai::node::MonoCamera>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto manipOut = pipeline.create<dai::node::XLinkOut>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

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
    dai::Device device(pipeline);
    // Start the pipeline
    device.startPipeline();

    // Queues
    auto qRight = device.getOutputQueue("right", 4, false);
    auto detections = device.getOutputQueue("detections", 4, false);

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](std::string name, auto frame, auto detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            int labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        auto inDet = detections->get<dai::ImgDetections>();
        auto detections = inDet->detections;
        cv::Mat frame = inRight->getCvFrame();

        displayFrame("right", frame, detections);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q')
            return 0;
    }
    return 0;
}
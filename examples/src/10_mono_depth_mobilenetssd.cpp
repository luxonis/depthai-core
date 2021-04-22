#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> flipRectified{true};

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
    dai::Pipeline p;

    // Define source
    auto camRight = p.create<dai::node::MonoCamera>();
    auto camLeft = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    auto manip = p.create<dai::node::ImageManip>();
    auto nn = p.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = p.create<dai::node::XLinkOut>();
    auto disparityOut = p.create<dai::node::XLinkOut>();
    auto xoutRight = p.create<dai::node::XLinkOut>();

    // Properties
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    stereo->setOutputRectified(true);
    stereo->setConfidenceThreshold(255);
    stereo->setRectifyEdgeFillColor(0);
    manip->initialConfig.setResize(300, 300);
    manip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);
    disparityOut->setStreamName("disparity");
    xoutRight->setStreamName("rectifiedRight");
    nnOut->setStreamName("nn");

    // Create outputs
    camRight->out.link(stereo->right);
    camLeft->out.link(stereo->left);
    stereo->rectifiedRight.link(manip->inputImage);
    stereo->disparity.link(disparityOut->input);
    manip->out.link(nn->input);
    manip->out.link(xoutRight->input);
    nn->out.link(nnOut->input);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    // Queuess
    auto qRight = d.getOutputQueue("rectifiedRight", 4, false);
    auto qDisparity = d.getOutputQueue("disparity", 4, false);
    auto qDet = d.getOutputQueue("nn", 4, false);

    // Add bounding boxes and text to the frame and show it to the user
    auto show = [](std::string name, auto frame, auto detections) {
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

    float disparity_multiplier = 255 / 95;
    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();
        auto inDisparity = qDisparity->get<dai::ImgFrame>();
        auto detections = inDet->detections;
        cv::Mat rightFrame = inRight->getCvFrame();
        cv::Mat disparityFrame = inDisparity->getCvFrame();

        if (flipRectified){
            cv::flip(rightFrame, rightFrame, 1);

            for(auto& detection : detections) {
                auto swap = detection.xmin;
                detection.xmin = 1 - detection.xmax;
                detection.xmax = 1 - swap;
            }
        }
        disparityFrame.convertTo(disparityFrame, CV_8UC1, disparity_multiplier);
        //Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        cv::applyColorMap(disparityFrame, disparityFrame, cv::COLORMAP_JET);
        show("disparity", disparityFrame, detections);
        show("rectified right", rightFrame, detections);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q')
            return 0;
    }
    return 0;
}
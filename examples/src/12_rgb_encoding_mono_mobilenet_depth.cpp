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
    dai::Pipeline pipeline;

    // Define source
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto camLeft = pipeline.create<dai::node::MonoCamera>();
    auto camRight = pipeline.create<dai::node::MonoCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    auto videoOut = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto disparityOut = pipeline.create<dai::node::XLinkOut>();
    auto manipOut = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    videoEncoder->setDefaultProfilePreset(1920, 1080, 30, dai::VideoEncoderProperties::Profile::H265_MAIN);
    camRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    camRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    camLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    camLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

    depth->setConfidenceThreshold(255);
    depth->setOutputRectified(true);
    depth->setRectifyMirrorFrame(false);
    depth->setRectifyEdgeFillColor(0);

    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);

    // The NN model expects BGR input-> By default ImageManip output type would be same as input (gray in this case)
    manip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);
    manip->initialConfig.setResize(300, 300);

    videoOut->setStreamName("h265");
    disparityOut->setStreamName("disparity");
    xoutRight->setStreamName("right");
    manipOut->setStreamName("manip");
    nnOut->setStreamName("nn");

    // Create outputs
    camRgb->video.link(videoEncoder->input);
    videoEncoder->bitstream.link(videoOut->input);
    camRight->out.link(xoutRight->input);
    camRight->out.link(depth->right);
    camLeft->out.link(depth->left);
    depth->disparity.link(disparityOut->input);
    depth->rectifiedRight.link(manip->inputImage);
    manip->out.link(nn->input);
    manip->out.link(manipOut->input);
    nn->out.link(nnOut->input);

    // Connect to device with above created pipeline
    dai::Device device(pipeline);
    // Start the pipeline
    device.startPipeline();

    // Queues
    int queueSize = 8;
    auto qRight = device.getOutputQueue("right", queueSize);
    auto qDisparity = device.getOutputQueue("disparity", queueSize);
    auto qManip = device.getOutputQueue("manip", queueSize);
    auto qDet = device.getOutputQueue("nn", queueSize);
    auto qRgbEnc = device.getOutputQueue("h265", 30, true);

    auto videoFile = std::ofstream("video.h265", std::ios::binary);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    cv::namedWindow("manip", cv::WINDOW_NORMAL);

    // Disparity range is 0..95, used for normalization
    float disparity_multiplier = 255 / 95;

    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        auto inManip = qManip->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();
        auto inDisparity = qDisparity->get<dai::ImgFrame>();
        auto detections = inDet->detections;

        cv::Mat frame = inRight->getCvFrame();
        cv::Mat frameManip = inManip->getCvFrame();
        cv::Mat frameDisparity = inDisparity->getCvFrame();

        auto out1 = qRgbEnc->get<dai::ImgFrame>();
        videoFile.write((char*)out1->getData().data(), out1->getData().size());

        if (flipRectified){
            cv::flip(frameDisparity, frameDisparity, 1);
        }
        frameDisparity.convertTo(frameDisparity, CV_8UC1, disparity_multiplier);
        cv::applyColorMap(frameDisparity, frameDisparity, cv::COLORMAP_JET);

        int offsetX = (camRight->getResolutionWidth() - camRight->getResolutionHeight()) / 2;
        auto color = cv::Scalar(255, 0, 0);
        for(auto& detection : detections) {
            int x1 = detection.xmin * camRight->getResolutionHeight() + offsetX;
            int y1 = detection.ymin * camRight->getResolutionHeight();
            int x2 = detection.xmax * camRight->getResolutionHeight() + offsetX;
            int y2 = detection.ymax * camRight->getResolutionHeight();

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
        // Show the right cam frame
        cv::imshow("right", frame);

        for(auto& detection : detections) {
            int x1 = detection.xmin * camRight->getResolutionHeight() + offsetX;
            int y1 = detection.ymin * camRight->getResolutionHeight();
            int x2 = detection.xmax * camRight->getResolutionHeight() + offsetX;
            int y2 = detection.ymax * camRight->getResolutionHeight();

            int labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frameDisparity, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frameDisparity, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frameDisparity, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the disparity frame
        cv::imshow("disparity", frameDisparity);

        for(auto& detection : detections) {
            int x1 = detection.xmin * frameManip.cols;
            int y1 = detection.ymin * frameManip.rows;
            int x2 = detection.xmax * frameManip.cols;
            int y2 = detection.ymax * frameManip.rows;

            int labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frameManip, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frameManip, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frameManip, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the manip frame
        cv::imshow("manip", frameManip);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q'){
            std::cout << "To view the encoded data, convert the stream file (.h265) into a video file (.mp4), using a command below:" << std::endl;
            std::cout << "ffmpeg -framerate 30 -i video.h265 -c copy video.mp4" << std::endl;
            return 0;
        }
    }
    return 0;
}
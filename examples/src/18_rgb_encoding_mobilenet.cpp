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
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto videoOut = pipeline.create<dai::node::XLinkOut>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setPreviewSize(300, 300);
    camRgb->setInterleaved(false);

    videoEncoder->setDefaultProfilePreset(1920, 1080, 30, dai::VideoEncoderProperties::Profile::H265_MAIN);

    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    videoOut->setStreamName("h265");
    xoutRgb->setStreamName("rgb");
    nnOut->setStreamName("nn");

    // Create outputs
    camRgb->video.link(videoEncoder->input);
    camRgb->preview.link(xoutRgb->input);
    camRgb->preview.link(nn->input);
    videoEncoder->bitstream.link(videoOut->input);
    nn->out.link(nnOut->input);

    // Connect to device with above created pipeline
    dai::Device device(pipeline);
    // Start the pipeline
    device.startPipeline();

    // Queues
    int queueSize = 8;
    auto qRgb = device.getOutputQueue("rgb", queueSize);
    auto qDet = device.getOutputQueue("nn", queueSize);
    auto qRgbEnc = device.getOutputQueue("h265", 30, true);

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

    auto videoFile = std::ofstream("video.h264", std::ios::binary);

    while(true) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();
        auto out = qRgbEnc->get<dai::ImgFrame>();
        auto detections = inDet->detections;
        cv::Mat frame = inRgb->getCvFrame();

        videoFile.write((char*)out->getData().data(), out->getData().size());

        displayFrame("rgb", frame, detections);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q'){
            std::cout << "To view the encoded data, convert the stream file (.h265) into a video file (.mp4), using a command below:" << std::endl;
            std::cout << "ffmpeg -framerate 30 -i video.h264 -c copy video.mp4" << std::endl;
            return 0;
        }
    }
    return 0;
}

#include <iostream>
#include <memory>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr auto MODEL_IN_WIDTH = 512;
static constexpr auto MODEL_IN_HEIGHT = 512;

int main(int argc, char** argv) {
    int camId = 0;
    if(argc > 1) {
        camId = std::stoi(argv[1]);
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto xin = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    nn->setXmlModelPath(MODEL_XML_PATH, MODEL_BIN_PATH);

    xin->setStreamName("nn_in");
    xout->setStreamName("nn_out");

    xin->setMaxDataSize(MODEL_IN_WIDTH * MODEL_IN_HEIGHT * 3);
    xin->setNumFrames(4);

    // Linking
    xin->out.link(nn->input);
    nn->out.link(xout->input);

    // Open Webcam
    cv::VideoCapture webcam(camId);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    cv::Mat frame;
    auto in = device.getInputQueue("nn_in");
    auto detections = device.getOutputQueue("nn_out");

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow("preview", frame);
    };

    std::vector<dai::ImgDetection> vdetections;

    while(true) {
        // data to send further
        auto tensor = std::make_shared<dai::Buffer>();

        // Read frame from webcam
        webcam >> frame;

        // crop and resize
        cv::Mat resizedFrame = resizeKeepAspectRatio(frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), cv::Scalar(0));
        std::vector<uint8_t> frameData;
        toPlanar(resizedFrame, frameData);
        tensor->setData(frameData);

        in->send(tensor);

        auto inDet = detections->get<dai::ImgDetections>();
        if(inDet) {
            vdetections = inDet->detections;
        }
        displayFrame(resizedFrame, vdetections);
        cv::imshow("preview", resizedFrame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
}


#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#define MODEL_IN_WIDTH 416
#define MODEL_IN_HEIGHT 416

#define MIN_SCORE 0.3

int main(int argc, char** argv) {
    using namespace std;

    int camId = 0;
    if(argc > 1) {
        camId = std::stoi(argv[1]);
    }

    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto nn = pipeline.create<dai::node::YoloDetectionNetwork>();
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

    // Detection network settings
    nn->setConfidenceThreshold(0.5f);
    nn->setNumClasses(80);
    nn->setCoordinateSize(4);  // What does this do?
    nn->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    nn->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    nn->setIouThreshold(0.5f);

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
        auto tensor = std::make_shared<dai::RawBuffer>();

        // Read frame from webcam
         webcam >> frame;

        // crop and resize
        cv::Mat resized_frame = resizeKeepAspectRatio(frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), cv::Scalar(0));
        // cv::Mat resized_frame;
        // cv::resize(frame, resized_frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT));

        toPlanar(resized_frame, tensor->data);
        // transform to BGR planar 300x300

        // tensor->data = std::vector<std::uint8_t>(frame.data, frame.data + frame.total());
        in->send(tensor);

        auto inDet = detections->tryGet<dai::ImgDetections>();
        if(inDet) {
            vdetections = inDet->detections;
        }
        displayFrame(resized_frame, vdetections);
        cv::imshow("preview", resized_frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
}

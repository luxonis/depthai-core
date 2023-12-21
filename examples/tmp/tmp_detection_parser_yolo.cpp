
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr auto MODEL_IN_WIDTH = 416;
static constexpr auto MODEL_IN_HEIGHT = 416;

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

int main(int argc, char** argv) {
    int camId = 0;
    if(argc > 1) {
        camId = std::stoi(argv[1]);
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto nn = pipeline.create<dai::node::YoloDetectionNetwork>();
    auto xin = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    nn->setXmlModelPath(MODEL_XML_PATH, MODEL_BIN_PATH);
    nn->setNumInferenceThreads(10);

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
    auto displayFrame = [](std::string name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = std::to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    std::vector<dai::ImgDetection> vDetections;

    using namespace std::chrono;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        // data to send further
        // auto tensor = std::make_shared<dai::RawBuffer>();
        dai::Buffer tensor;

        // Read frame from webcam
        webcam >> frame;

        // crop and resize
        cv::Mat resizedFrame = resizeKeepAspectRatio(frame, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), cv::Scalar(0));

        std::vector<uint8_t> data;
        toPlanar(resizedFrame, data);
        tensor.setData(data);

        in->send(tensor);

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto inDet = detections->get<dai::ImgDetections>();
        if(inDet) {
            vDetections = inDet->detections;
            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(resizedFrame, fpsStr.str(), cv::Point(2, MODEL_IN_HEIGHT - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 255, 255));
        }
        displayFrame("nn_out", resizedFrame, vDetections);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
}

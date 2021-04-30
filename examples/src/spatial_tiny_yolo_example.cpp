#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

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

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    // create nodes
    auto colorCam = p.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = p.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();

    // create xlink connections
    auto xoutRgb = p.create<dai::node::XLinkOut>();
    auto xoutNN = p.create<dai::node::XLinkOut>();
    auto xoutBoundingBoxDepthMapping = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutNN->setStreamName("detections");
    xoutBoundingBoxDepthMapping->setStreamName("boundingBoxDepthMapping");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    /// setting node configs
    stereo->setConfidenceThreshold(255);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    else
        colorCam->preview.link(xoutRgb->input);

    spatialDetectionNetwork->out.link(xoutNN->input);
    spatialDetectionNetwork->boundingBoxMapping.link(xoutBoundingBoxDepthMapping->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline p = createNNPipeline(nnPath);

    // Connect to device with above created pipeline
    dai::Device d(p);
    // Start the pipeline
    d.startPipeline();

    auto preview = d.getOutputQueue("preview", 4, false);
    auto detections = d.getOutputQueue("detections", 4, false);
    auto xoutBoundingBoxDepthMapping = d.getOutputQueue("boundingBoxDepthMapping", 4, false);
    auto depthQueue = d.getOutputQueue("depth", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto det = detections->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        auto dets = det->detections;

        cv::Mat depthFrame = depth->getFrame();
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        if(!dets.empty()) {
            auto boundingBoxMapping = xoutBoundingBoxDepthMapping->get<dai::SpatialLocationCalculatorConfig>();
            auto roiDatas = boundingBoxMapping->getConfigData();

            for(auto roiData : roiDatas) {
                auto roi = roiData.roi;
                roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
                auto topLeft = roi.topLeft();
                auto bottomRight = roi.bottomRight();
                auto xmin = (int)topLeft.x;
                auto ymin = (int)topLeft.y;
                auto xmax = (int)bottomRight.x;
                auto ymax = (int)bottomRight.y;

                cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            }
        }
        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat frame = imgFrame->getCvFrame();

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
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream depthX;
            depthX << "X: " << (int)d.spatialCoordinates.x << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)d.spatialCoordinates.y << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)d.spatialCoordinates.z << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

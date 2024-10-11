#include <atomic>
#include <chrono>
#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr auto FPS = 15;

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

static std::atomic<bool> syncNN{true};

int main(int argc, char** argv) {
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto tof = pipeline.create<dai::node::ToF>();
    auto camTof = pipeline.create<dai::node::Camera>();
    auto imageAlign = pipeline.create<dai::node::ImageAlign>();

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");
    xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    camRgb->setPreviewSize(416, 416);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    camRgb->setFps(FPS);

    camTof->setFps(FPS);
    camTof->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camTof->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    imageAlign->setOutputSize(640, 400);

    spatialDetectionNetwork->setBlobPath(BLOB_PATH);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    camTof->raw.link(tof->input);
    tof->depth.link(imageAlign->input);

    camRgb->preview.link(spatialDetectionNetwork->input);
    if(syncNN) {
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    } else {
        camRgb->preview.link(xoutRgb->input);
    }

    spatialDetectionNetwork->out.link(xoutNN->input);

    camRgb->isp.link(imageAlign->inputAlignTo);
    imageAlign->outputAligned.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    dai::Device device(pipeline);

    auto previewQueue = device.getOutputQueue("rgb", 4, false);
    auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
    auto depthQueue = device.getOutputQueue("depth", 4, false);

    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    while(true) {
        auto inPreview = previewQueue->get<dai::ImgFrame>();
        auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        cv::Mat frame = inPreview->getCvFrame();
        cv::Mat depthFrame = depth->getFrame();  // depthFrame values are in millimeters

        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - startTime);
        if(elapsed > std::chrono::seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto detections = inDet->detections;

        for(const auto& detection : detections) {
            auto roiData = detection.boundingBoxMapping;
            auto roi = roiData.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto topLeft = roi.topLeft();
            auto bottomRight = roi.bottomRight();
            auto xmin = static_cast<int>(topLeft.x);
            auto ymin = static_cast<int>(topLeft.y);
            auto xmax = static_cast<int>(bottomRight.x);
            auto ymax = static_cast<int>(bottomRight.y);
            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, 1);

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
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream depthX;
            depthX << "X: " << static_cast<int>(detection.spatialCoordinates.x) << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthY;
            depthY << "Y: " << static_cast<int>(detection.spatialCoordinates.y) << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthZ;
            depthZ << "Z: " << static_cast<int>(detection.spatialCoordinates.z) << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rgb", frame);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}

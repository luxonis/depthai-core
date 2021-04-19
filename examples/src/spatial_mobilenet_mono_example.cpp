#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static bool syncNN = true;
static bool flipRectified = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto spatialDetectionNetwork = p.create<dai::node::MobileNetSpatialDetectionNetwork>();
    auto imageManip = p.create<dai::node::ImageManip>();

    auto nnOut = p.create<dai::node::XLinkOut>();
    auto depthRoiMap = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");
    depthRoiMap->setStreamName("boundingBoxDepthMapping");
    xoutDepth->setStreamName("depth");

    imageManip->initialConfig.setResize(300, 300);
    // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    imageManip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->setConfidenceThreshold(255);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->rectifiedRight.link(imageManip->inputImage);

    // testing MobileNet DetectionNetwork
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.7);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    spatialDetectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    imageManip->out.link(spatialDetectionNetwork->input);
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xlinkOut->input);
    else
        imageManip->out.link(xlinkOut->input);

    spatialDetectionNetwork->out.link(nnOut->input);
    spatialDetectionNetwork->boundingBoxMapping.link(depthRoiMap->input);

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
    auto depthRoiMap = d.getOutputQueue("boundingBoxDepthMapping", 4, false);
    auto depthQueue = d.getOutputQueue("depth", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    while(1) {
        auto inRectifiedRight = preview->get<dai::ImgFrame>();
        auto det = detections->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto dets = det->detections;

        cv::Mat depthFrame = depth->getFrame();
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        if(!dets.empty()) {
            auto boundingBoxMapping = depthRoiMap->get<dai::SpatialLocationCalculatorConfig>();
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

        cv::Mat rectifiedRight = inRectifiedRight->getCvFrame();

        if(flipRectified) cv::flip(rectifiedRight, rectifiedRight, 1);

        for(auto& d : dets) {
            if(flipRectified) {
                auto swap = d.xmin;
                d.xmin = 1 - d.xmax;
                d.xmax = 1 - swap;
            }
            int x1 = d.xmin * rectifiedRight.cols;
            int y1 = d.ymin * rectifiedRight.rows;
            int x2 = d.xmax * rectifiedRight.cols;
            int y2 = d.ymax * rectifiedRight.rows;

            int labelIndex = d.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(rectifiedRight, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << d.confidence * 100;
            cv::putText(rectifiedRight, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream depthX;
            depthX << "X: " << (int)d.spatialCoordinates.x << " mm";
            cv::putText(rectifiedRight, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)d.spatialCoordinates.y << " mm";
            cv::putText(rectifiedRight, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)d.spatialCoordinates.z << " mm";
            cv::putText(rectifiedRight, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(rectifiedRight, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(rectifiedRight, fpsStr.str(), cv::Point(2, rectifiedRight.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rectified right", rectifiedRight);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

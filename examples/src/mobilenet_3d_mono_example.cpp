#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background",  "aeroplane", "bicycle", "bird",      "boat",   "bottle",      "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog",       "horse",   "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto detectionNetwork = p.create<dai::node::MobileNetDetectionNetworkDepth>();
    auto imageManip = p.create<dai::node::ImageManip>();
    auto nnOut = p.create<dai::node::XLinkOut>();
    auto depthRoiMap = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");
    depthRoiMap->setStreamName("depthRoiMap");
    xoutDepth->setStreamName("depth");


    imageManip->initialConfig.setResize(300, 300);
    //The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    imageManip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);



    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck = false;
    bool extended = false;
    bool subpixel = false;

    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(255);

    // stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    monoRight->out.link(imageManip->inputImage);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBoundingBoxScaleFactor(0.7);
    detectionNetwork->setDepthLowerThreshold(100);
    detectionNetwork->setDepthUpperThreshold(5000);

    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    imageManip->out.link(detectionNetwork->input);
    if(syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        imageManip->out.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    detectionNetwork->passthroughRoi.link(depthRoiMap->input);

    stereo->depth.link(detectionNetwork->inputDepth);
    stereo->depth.link(xoutDepth->input);

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

    cv::Mat frame;
    auto preview = d.getOutputQueue("preview", 4, false);
    auto detections = d.getOutputQueue("detections", 4, false);
    auto depthRoiMap = d.getOutputQueue("depthRoiMap", 4, false);
    auto depthQueue = d.getOutputQueue("depth", 4, false);

    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    while(1) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto det = detections->get<dai::ImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        auto dets = det->detections;

        cv::Mat depthFrame = cv::Mat(depth->getHeight(), depth->getWidth(), CV_16UC1, depth->getData().data());

        cv::Mat depthFrameColor = cv::Mat(depth->getHeight(), depth->getWidth(), CV_8UC1);
        cv::normalize(depthFrame, depthFrameColor, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        if(!dets.empty()) {
            auto passthroughRoi = depthRoiMap->get<dai::DepthCalculatorConfig>();
            auto roiDatas = passthroughRoi->getConfigData();

            for(auto roiData : roiDatas) {
                auto roi = roiData.roi;
                auto xmin = (int)(roi.xmin * depth->getWidth());
                auto ymin = (int)(roi.ymin * depth->getHeight());
                auto xmax = (int)(roi.xmax * depth->getWidth());
                auto ymax = (int)(roi.ymax * depth->getHeight());

                cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            }
        }
        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = (float)counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        if(imgFrame) {
            frame = toMat(imgFrame->getData(), imgFrame->getWidth(), imgFrame->getHeight(), 3, 1);
        }

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
            confStr << std::fixed << std::setprecision(2) << d.confidence*100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream depthX;
            depthX << "X: " << (int)d.xdepth << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)d.ydepth << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)d.zdepth << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight()-4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}
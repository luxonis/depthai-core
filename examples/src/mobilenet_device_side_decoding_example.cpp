#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static std::string label_map[] = {"background",  "aeroplane", "bicycle", "bird",      "boat",   "bottle",      "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog",       "horse",   "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};

static bool syncNN = true;

dai::Pipeline createNNPipeline(std::string nnPath) {
    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    auto detectionNetwork = p.create<dai::node::MobileNetDetectionNetworkDepth>();
    auto nnOut = p.create<dai::node::XLinkOut>();
    auto depthRoiMap = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");
    depthRoiMap->setStreamName("depthRoiMap");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(30);

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->setFps(30);
    monoRight->setFps(30);
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

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    detectionNetwork->passthroughRoi.link(depthRoiMap->input);

    stereo->depth.link(detectionNetwork->inputDepth);
    stereo->depth.link(xoutDepth->input);

    return p;
}

int main(int argc, char** argv) {
    using namespace std;
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

    auto start_time = std::chrono::steady_clock::now();
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
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_OCEAN);

        if(!dets.empty()) {
            auto passthroughRoi = depthRoiMap->get<dai::DepthCalculatorConfig>();
            auto roiData = passthroughRoi->getConfigData();

            for(auto roiData : roiData) {
                auto roi = roiData.roi;
                auto xmin = (int)(roi.xmin * depth->getWidth());
                auto ymin = (int)(roi.ymin * depth->getHeight());
                auto xmax = (int)(roi.xmax * depth->getWidth());
                auto ymax = (int)(roi.ymax * depth->getHeight());

                cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
                // std::stringstream s;
                // s << std::fixed << std::setprecision(2) << depthData.depth_avg;
                // cv::putText(frame, s.str(), cv::Point(xmin + 10, ymin + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            }
        }
        counter++;
        auto current_time = std::chrono::steady_clock::now();
        float elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.;
        if(elapsed_s > 1) {
            fps = counter / elapsed_s;
            counter = 0;
            start_time = current_time;
        }

        if(imgFrame) {
            frame = toMat(imgFrame->getData(), imgFrame->getWidth(), imgFrame->getHeight(), 3, 1);
        }

        for(const auto& d : dets) {
            int x1 = d.xmin * frame.cols;
            int y1 = d.ymin * frame.rows;
            int x2 = d.xmax * frame.cols;
            int y2 = d.ymax * frame.rows;

            int label_index = d.label;
            std::string label_str = to_string(label_index);
            if(label_index < sizeof(label_map) / sizeof(label_map[0])) {
                label_str = label_map[label_index];
            }
            cv::putText(frame, label_str, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            char conf_str[10];
            snprintf(conf_str, sizeof conf_str, "%.2f", d.confidence * 100);
            cv::putText(frame, conf_str, cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream depth_x;
            depth_x << std::fixed << std::setprecision(2) << d.xdepth;
            cv::putText(frame, depth_x.str(), cv::Point(x1 + 10, y1 + 60), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depth_y;
            depth_y << std::fixed << std::setprecision(2) << d.ydepth;
            cv::putText(frame, depth_y.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depth_z;
            depth_z << std::fixed << std::setprecision(2) << d.zdepth;
            cv::putText(frame, depth_z.str(), cv::Point(x1 + 10, y1 + 100), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        char fps_str[15];
        snprintf(fps_str, sizeof fps_str, "NN fps: %.2f", fps);
        cv::putText(frame, fps_str, cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("preview", frame);
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}
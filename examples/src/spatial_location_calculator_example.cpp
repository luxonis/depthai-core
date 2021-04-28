

#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr float stepSize = 0.05;

int main() {
    using namespace std;

    dai::Pipeline p;

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    auto spatialDataCalculator = p.create<dai::node::SpatialLocationCalculator>();

    auto xoutDepth = p.create<dai::node::XLinkOut>();
    auto xoutSpatialData = p.create<dai::node::XLinkOut>();
    auto xinSpatialCalcConfig = p.create<dai::node::XLinkIn>();

    // XLinkOut
    xoutDepth->setStreamName("depth");
    xoutSpatialData->setStreamName("spatialData");
    xinSpatialCalcConfig->setStreamName("spatialCalcConfig");

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    bool lrcheck = false;
    bool subpixel = false;

    // StereoDepth
    stereo->setConfidenceThreshold(255);

    // stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    spatialDataCalculator->passthroughDepth.link(xoutDepth->input);
    stereo->depth.link(spatialDataCalculator->inputDepth);

    dai::Point2f topLeft(0.4f, 0.4f);
    dai::Point2f bottomRight(0.6f, 0.6f);

    spatialDataCalculator->setWaitForConfigInput(false);
    dai::SpatialLocationCalculatorConfigData config;
    config.depthThresholds.lowerThreshold = 100;
    config.depthThresholds.upperThreshold = 10000;
    config.roi = dai::Rect(topLeft, bottomRight);
    spatialDataCalculator->initialConfig.addROI(config);
    spatialDataCalculator->out.link(xoutSpatialData->input);
    xinSpatialCalcConfig->out.link(spatialDataCalculator->inputConfig);

    // CONNECT TO DEVICE
    dai::Device d(p);
    d.startPipeline();

    auto depthQueue = d.getOutputQueue("depth", 8, false);
    auto spatialCalcQueue = d.getOutputQueue("spatialData", 8, false);
    auto spatialCalcConfigInQueue = d.getInputQueue("spatialCalcConfig");

    cv::Mat depthFrame;
    auto color = cv::Scalar(255, 255, 255);
    std::cout << "Use WASD keys to move ROI!" << std::endl;

    while(1) {
        auto depth = depthQueue->get<dai::ImgFrame>();

        cv::Mat depthFrame = depth->getFrame();
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        auto spatialData = spatialCalcQueue->get<dai::SpatialLocationCalculatorData>()->getSpatialLocations();
        for(auto depthData : spatialData) {
            auto roi = depthData.config.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto xmin = (int)roi.topLeft().x;
            auto ymin = (int)roi.topLeft().y;
            auto xmax = (int)roi.bottomRight().x;
            auto ymax = (int)roi.bottomRight().y;

            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            std::stringstream depthX;
            depthX << "X: " << (int)depthData.spatialCoordinates.x << " mm";
            cv::putText(depthFrameColor, depthX.str(), cv::Point(xmin + 10, ymin + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)depthData.spatialCoordinates.y << " mm";
            cv::putText(depthFrameColor, depthY.str(), cv::Point(xmin + 10, ymin + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)depthData.spatialCoordinates.z << " mm";
            cv::putText(depthFrameColor, depthZ.str(), cv::Point(xmin + 10, ymin + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        cv::imshow("depth", depthFrameColor);

        bool newConfig = false;
        int key = cv::waitKey(1);
        switch(key) {
            case 'q':
                return 0;
                break;
            case 'w':
                if(topLeft.y - stepSize >= 0) {
                    topLeft.y -= stepSize;
                    bottomRight.y -= stepSize;
                    newConfig = true;
                }
                break;
            case 'a':
                if(topLeft.x - stepSize >= 0) {
                    topLeft.x -= stepSize;
                    bottomRight.x -= stepSize;
                    newConfig = true;
                }
                break;
            case 's':
                if(bottomRight.y + stepSize <= 1) {
                    topLeft.y += stepSize;
                    bottomRight.y += stepSize;
                    newConfig = true;
                }
                break;
            case 'd':
                if(bottomRight.x + stepSize <= 1) {
                    topLeft.x += stepSize;
                    bottomRight.x += stepSize;
                    newConfig = true;
                }
                break;
            default:
                break;
        }
        if(newConfig) {
            config.roi = dai::Rect(topLeft, bottomRight);
            dai::SpatialLocationCalculatorConfig cfg;
            cfg.addROI(config);
            spatialCalcConfigInQueue->send(cfg);
        }
    }
}



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

    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck = false;
    bool subpixel = false;

    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(255);

    // stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    spatialDataCalculator->passthroughDepth.link(xoutDepth->input);
    stereo->depth.link(spatialDataCalculator->inputDepth);

    float bbXmin = 0.4f;
    float bbXmax = 0.6f;
    float bbYmin = 0.4f;
    float bbYmax = 0.6f;

    spatialDataCalculator->setWaitForConfigInput(false);
    dai::SpatialLocationCalculatorConfigData config;
    config.depthThresholds.lowerThreshold = 100;
    config.depthThresholds.upperThreshold = 10000;
    config.roi = dai::Rect(bbXmin, bbYmin, bbXmax, bbYmax);
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
            auto xmin = (int)(roi.xmin * depth->getWidth());
            auto ymin = (int)(roi.ymin * depth->getHeight());
            auto xmax = (int)(roi.xmax * depth->getWidth());
            auto ymax = (int)(roi.ymax * depth->getHeight());

            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            std::stringstream s;
            s << "Avg: " << (int)depthData.depthAverage << " mm";
            cv::putText(depthFrameColor, s.str(), cv::Point(xmin + 10, ymin + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthX;
            depthX << "X: " << (int)depthData.spatialCoordinates.x << " mm";
            cv::putText(depthFrameColor, depthX.str(), cv::Point(xmin + 10, ymin + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)depthData.spatialCoordinates.y << " mm";
            cv::putText(depthFrameColor, depthY.str(), cv::Point(xmin + 10, ymin + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)depthData.spatialCoordinates.z << " mm";
            cv::putText(depthFrameColor, depthZ.str(), cv::Point(xmin + 10, ymin + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        cv::imshow("depth", depthFrameColor);

        bool newConfig = false;
        int key = cv::waitKey(1);
        switch(key) {
            case 'q':
                return 0;
                break;
            case 'w':
                if(bbYmin - stepSize >= 0) {
                    bbYmin -= stepSize;
                    bbYmax -= stepSize;
                    newConfig = true;
                }
                break;
            case 'a':
                if(bbXmin - stepSize >= 0) {
                    bbXmin -= stepSize;
                    bbXmax -= stepSize;
                    newConfig = true;
                }
                break;
            case 's':
                if(bbYmax + stepSize <= 1) {
                    bbYmin += stepSize;
                    bbYmax += stepSize;
                    newConfig = true;
                }
                break;
            case 'd':
                if(bbXmax + stepSize <= 1) {
                    bbXmin += stepSize;
                    bbXmax += stepSize;
                    newConfig = true;
                }
                break;
            default:
                break;
        }
        if(newConfig) {
            config.roi = dai::Rect(bbXmin, bbYmin, bbXmax, bbYmax);
            dai::SpatialLocationCalculatorConfig cfg;
            cfg.addROI(config);
            spatialCalcConfigInQueue->send(cfg);
        }
    }
}

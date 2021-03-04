

#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    dai::Pipeline p;

    auto monoLeft = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto stereo = p.create<dai::node::StereoDepth>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();
    auto depthCalculator = p.create<dai::node::DepthCalculator>();
    auto xoutDepthCalc = p.create<dai::node::XLinkOut>();
    // auto xinDepthCalcConfig = p.create<dai::node::XLinkIn>();

    // XLinkOut
    xoutDepth->setStreamName("depth");
    xoutDepthCalc->setStreamName("depthCalcAvg");
    // xinDepthCalcConfig->setStreamName("depthCalcConfig");

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck = true;
    bool extended = false;
    bool subpixel = true;

    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);

    // stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->depth.link(xoutDepth->input);
    stereo->depth.link(depthCalculator->depthInput);

    depthCalculator->setWaitForConfigInput(true);
    dai::DepthCalculatorConfigData config;
    config.lower_threshold = 100;
    config.upper_threshold = 10000;
    config.roi = dai::Rect(0.4, 0.4, 0.45, 0.45);
    depthCalculator->initialConfig.addROI(config);
    depthCalculator->out.link(xoutDepthCalc->input);
    // xinDepthCalcConfig->out.link(depthCalculator->inputConfig);

    // CONNECT TO DEVICE
    dai::Device d(p);
    d.startPipeline();

    auto depthQueue = d.getOutputQueue("depth", 8, false);
    auto depthCalcQueue = d.getOutputQueue("depthCalcAvg", 8, false);
    // auto depthCalcConfigInQueue = d.getInputQueue("depthCalcConfig");

    cv::Mat frame;
    auto color = cv::Scalar(255, 255, 255);
    int iteration = 0;
    while(1) {

        // dai::DepthCalculatorConfigData config;
        // config.lower_threshold = 100;
        // config.upper_threshold = 5000;
        // float pt = (iteration % 10)/10.f;
        // config.roi = dai::Rect(pt, pt, 1, 1);
        // iteration++;
        // dai::DepthCalculatorConfig cfg;
        // cfg.addROI(config);
        // depthCalcConfigInQueue->send(cfg);

        auto depth = depthQueue->get<dai::ImgFrame>();
        frame = cv::Mat(depth->getHeight(), depth->getWidth(), CV_16UC1, depth->getData().data());

        auto depthCalcData = depthCalcQueue->get<dai::DepthCalculatorData>()->getDepthData();
        for(auto depthData : depthCalcData) {
            auto roi = depthData.config.roi;
            auto xmin = (int)(roi.xmin * depth->getWidth());
            auto ymin = (int)(roi.ymin * depth->getHeight());
            auto xmax = (int)(roi.xmax * depth->getWidth());
            auto ymax = (int)(roi.ymax * depth->getHeight());

            cv::rectangle(frame, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            std::stringstream s;
            s << std::fixed << std::setprecision(2) << depthData.depth_avg;
            cv::putText(frame, s.str(), cv::Point(xmin + 10, ymin + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        cv::imshow("depth", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
}

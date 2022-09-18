#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialLocationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();

    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutSpatialData = pipeline.create<dai::node::XLinkOut>();

    xoutDepth->setStreamName("depth");
    xoutSpatialData->setStreamName("spatialData");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(true);
    spatialLocationCalculator->inputConfig.setWaitForMessage(false);

    // Create 10 ROIs
    for(int i = 0; i < 10; i++) {
        dai::SpatialLocationCalculatorConfigData config;
        config.depthThresholds.lowerThreshold = 200;
        config.depthThresholds.upperThreshold = 10000;
        config.roi = dai::Rect(dai::Point2f(i * 0.1, 0.45), dai::Point2f((i + 1) * 0.1, 0.55));
        spatialLocationCalculator->initialConfig.addROI(config);
    }

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    spatialLocationCalculator->passthroughDepth.link(xoutDepth->input);
    stereo->depth.link(spatialLocationCalculator->inputDepth);

    spatialLocationCalculator->out.link(xoutSpatialData->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the depth frames from the outputs defined above
    auto depthQueue = device.getOutputQueue("depth", 4, false);
    auto spatialCalcQueue = device.getOutputQueue("spatialData", 3, false);

    auto color = cv::Scalar(255, 255, 255);

    while(true) {
        auto inDepth = depthQueue->get<dai::ImgFrame>();

        cv::Mat depthFrame = inDepth->getFrame();  // depthFrame values are in millimeters
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

            auto depthMin = depthData.depthMin;
            auto depthMax = depthData.depthMax;

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
        // Show the frame
        cv::imshow("depth", depthFrameColor);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}

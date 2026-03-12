#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

// Visualization helper
void showDepth(const cv::Mat& depthFrame, const std::string& windowName = "Depth", int minDistance = 500, int maxDistance = 5000) {
    cv::Mat displayFrame;
    depthFrame.convertTo(displayFrame, CV_8UC1, 255.0 / maxDistance);

    cv::Mat colorMap;
    cv::applyColorMap(displayFrame, colorMap, cv::COLORMAP_TURBO);

    cv::imshow(windowName, colorMap);
}

void botchCalibration(std::shared_ptr<dai::Device> device) {
    auto calibHandler = device->readCalibration();

    std::vector<std::vector<float>> rotation = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    std::vector<float> tvec = {-7.5, 0., 0.};

    // Set extrinsics using the existing rotation matrix and the new translation
    calibHandler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, rotation, tvec, tvec);

    device->setCalibration(calibHandler);
}

int main() {
    dai::Pipeline pipeline;

    // Create device
    auto device = pipeline.getDefaultDevice();
    botchCalibration(device);

    // Nodes
    auto camLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // AutoCalibration node
    auto dcWorker = pipeline.create<dai::node::AutoCalibration>();
    dcWorker->build(camLeft, camRight);

    auto config = dcWorker->initialConfig;
    config->maxIterations = 2;
    config->sleepingTime = 10;
    config->flashCalibration = false;
    config->mode = dai::AutoCalibrationConfig::CONTINUOUS;
    config->validationSetSize = 5;
    config->dataConfidenceThreshold = 0.7;

    // Links
    camLeft->requestOutput({1280, 800})->link(stereo->left);
    camRight->requestOutput({1280, 800})->link(stereo->right);

    // Queues
    auto workerOutputQueue = dcWorker->output.createOutputQueue();
    auto stereoOut = stereo->depth.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto workerOutput = workerOutputQueue->tryGet<dai::AutoCalibrationResult>();
        if(workerOutput != nullptr) {
            if(workerOutput->passed) {
                std::cout << "Passed. Confidence: " << workerOutput->dataConfidence << std::endl;
            } else {
                std::cout << "Did not pass." << std::endl;
            }
        }

        auto depth = stereoOut->get<dai::ImgFrame>();
        showDepth(depth->getCvFrame(), "Depth", 500, 5000);

        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"

// Visualization helper
void showDepth(const cv::Mat& depthFrame, const std::string& windowName = "Depth", int minDistance = 500, int maxDistance = 5000) {
    if(maxDistance <= minDistance) return;

    cv::Mat clipped = depthFrame.clone();
    clipped.setTo(minDistance, depthFrame < minDistance);
    clipped.setTo(maxDistance, depthFrame > maxDistance);

    cv::Mat displayFrame;
    double scale = 255.0 / (maxDistance - minDistance);
    double offset = -minDistance * scale;
    clipped.convertTo(displayFrame, CV_8UC1, scale, offset);

    cv::Mat colorMap;
    cv::applyColorMap(displayFrame, colorMap, cv::COLORMAP_TURBO);

    cv::imshow(windowName, colorMap);
}

std::tuple<double, double, double> rotationMatrixToEulerAngles(const cv::Matx33d& rotationMatrix) {
    constexpr double kPi = 3.14159265358979323846;
    const double sy = std::sqrt(rotationMatrix(0, 0) * rotationMatrix(0, 0) + rotationMatrix(1, 0) * rotationMatrix(1, 0));
    const bool singular = sy < 1e-6;

    double xAngle;
    double yAngle;
    double zAngle;
    if(!singular) {
        xAngle = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
        yAngle = std::atan2(-rotationMatrix(2, 0), sy);
        zAngle = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    } else {
        xAngle = std::atan2(-rotationMatrix(1, 2), rotationMatrix(1, 1));
        yAngle = std::atan2(-rotationMatrix(2, 0), sy);
        zAngle = 0.0;
    }

    return {xAngle * 180.0 / kPi, yAngle * 180.0 / kPi, zAngle * 180.0 / kPi};
}

cv::Matx33d eulerAnglesToRotationMatrix(double phi, double theta, double psi) {
    constexpr double kPi = 3.14159265358979323846;
    const double phiRad = phi * kPi / 180.0;
    const double thetaRad = theta * kPi / 180.0;
    const double psiRad = psi * kPi / 180.0;

    const cv::Matx33d rx(1.0, 0.0, 0.0, 0.0, std::cos(phiRad), -std::sin(phiRad), 0.0, std::sin(phiRad), std::cos(phiRad));
    const cv::Matx33d ry(std::cos(thetaRad), 0.0, std::sin(thetaRad), 0.0, 1.0, 0.0, -std::sin(thetaRad), 0.0, std::cos(thetaRad));
    const cv::Matx33d rz(std::cos(psiRad), -std::sin(psiRad), 0.0, std::sin(psiRad), std::cos(psiRad), 0.0, 0.0, 0.0, 1.0);

    return rz * ry * rx;
}

void botchCalibration(std::shared_ptr<dai::Device> device) {
    auto calibHandler = device->readCalibration();
    const auto extrinsics = calibHandler.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);

    if(extrinsics.size() < 3 || extrinsics[0].size() < 4 || extrinsics[1].size() < 4 || extrinsics[2].size() < 4) {
        throw std::runtime_error("Invalid CAM_B->CAM_C extrinsics shape, expected at least 3x4.");
    }

    const cv::Matx33d rCurrent(static_cast<double>(extrinsics[0][0]),
                               static_cast<double>(extrinsics[0][1]),
                               static_cast<double>(extrinsics[0][2]),
                               static_cast<double>(extrinsics[1][0]),
                               static_cast<double>(extrinsics[1][1]),
                               static_cast<double>(extrinsics[1][2]),
                               static_cast<double>(extrinsics[2][0]),
                               static_cast<double>(extrinsics[2][1]),
                               static_cast<double>(extrinsics[2][2]));

    auto [phi, theta, psi] = rotationMatrixToEulerAngles(rCurrent);
    const double phiBotched = phi + 0.15;
    const double thetaBotched = theta + 0.10;
    const double psiBotched = psi;

    const cv::Matx33d rBotched = eulerAnglesToRotationMatrix(phiBotched, thetaBotched, psiBotched);

    std::vector<float> t = {extrinsics[0][3], extrinsics[1][3], extrinsics[2][3]};
    std::vector<std::vector<float>> r = {{static_cast<float>(rBotched(0, 0)), static_cast<float>(rBotched(0, 1)), static_cast<float>(rBotched(0, 2))},
                                         {static_cast<float>(rBotched(1, 0)), static_cast<float>(rBotched(1, 1)), static_cast<float>(rBotched(1, 2))},
                                         {static_cast<float>(rBotched(2, 0)), static_cast<float>(rBotched(2, 1)), static_cast<float>(rBotched(2, 2))}};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Original Euler deg (x, y, z): (" << phi << ", " << theta << ", " << psi << ")" << std::endl;
    std::cout << "Botched  Euler deg (x, y, z): (" << phiBotched << ", " << thetaBotched << ", " << psiBotched << ")" << std::endl;
    std::cout << "Keeping translation vector (cm): [" << t[0] << ", " << t[1] << ", " << t[2] << "]" << std::endl;

    calibHandler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, r, t, t);
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
    config->dataConfidenceThreshold = 0.3;

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
                std::cout << "Passed" << std::endl;
                std::cout << "dataConfidence = " << workerOutput->dataConfidence << std::endl;
                std::cout << "calibrationConfidence = " << workerOutput->calibrationConfidence << std::endl;
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

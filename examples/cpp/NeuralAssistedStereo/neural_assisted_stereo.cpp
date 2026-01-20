#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/NeuralAssistedStereo.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr float FPS = 20.0f;
// Nicely visualize a depth map.
// The input depthFrame is assumed to be the raw disparity (CV_16UC1 or similar)
// received from the DepthAI pipeline.
void showDepth(const cv::Mat& depthFrameIn,
               const std::string& windowName = "Depth",
               int minDistance = 500,
               int maxDistance = 5000,
               int colormap = cv::COLORMAP_TURBO,
               bool useLog = false) {
    cv::Mat depthFrame = depthFrameIn.clone();

    cv::Mat floatFrame;
    depthFrame.convertTo(floatFrame, CV_32FC1);

    // # Optionally apply log scaling
    if(useLog) {
        // depthFrame = np.log(depthFrame + 1)
        cv::log(floatFrame + 1, floatFrame);
    }

    cv::Mat upperClamped;
    cv::min(floatFrame, maxDistance, upperClamped);

    cv::Mat clippedFrame;
    cv::max(upperClamped, minDistance, clippedFrame);

    double alpha = 255.0 / maxDistance;
    clippedFrame.convertTo(clippedFrame, CV_8U, alpha);

    cv::Mat depthColor;
    cv::applyColorMap(clippedFrame, depthColor, colormap);

    cv::imshow(windowName, depthColor);
}

int main() {
    // 1. Create device and pipeline
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);
    if(!device->isNeuralDepthSupported()) {
        std::cout << "Exiting NeuralAssistedStereo example: device doesn't support NeuralDepth.\n";
        return 0;
    }

    // 2. Define nodes
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, FPS);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, FPS);
    auto monoRightOut = monoRight->requestFullResolutionOutput();
    auto monoLeftOut = monoLeft->requestFullResolutionOutput();

    auto neuralAssistedStereo = pipeline.create<dai::node::NeuralAssistedStereo>()->build(*monoLeftOut, *monoRightOut, dai::DeviceModelZoo::NEURAL_DEPTH_NANO);

    // 6. Get output queue
    auto disparityQueue = neuralAssistedStereo->disparity.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto disparityPacket = disparityQueue->get<dai::ImgFrame>();
        showDepth(disparityPacket->getCvFrame(), "Depth", 100, 6000, cv::COLORMAP_TURBO, false);
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
    return 0;
}

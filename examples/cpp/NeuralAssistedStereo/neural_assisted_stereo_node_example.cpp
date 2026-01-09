#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/NeuralAssistedStereo.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

// Nicely visualize a depth map.
// The input depthFrame is assumed to be the raw disparity (CV_16UC1 or similar)
// received from the DepthAI pipeline.
void showDepth(const cv::Mat& depthFrame_in,
               const std::string& windowName = "Depth",
               int minDistance = 500,
               int maxDistance = 5000,
               int colormap = cv::COLORMAP_TURBO,
               bool useLog = false) {
    cv::Mat depthFrame = depthFrame_in.clone();

    cv::Mat float_frame;
    depthFrame.convertTo(float_frame, CV_32FC1);

    // # Optionally apply log scaling
    if(useLog) {
        // depthFrame = np.log(depthFrame + 1)
        cv::log(float_frame + 1, float_frame);
    }

    cv::Mat upper_clamped;
    cv::min(float_frame, maxDistance, upper_clamped);

    cv::Mat clipped_frame;
    cv::max(upper_clamped, minDistance, clipped_frame);

    double alpha = 255.0 / maxDistance;
    clipped_frame.convertTo(clipped_frame, CV_8U, alpha);

    cv::Mat depth_color;
    cv::applyColorMap(clipped_frame, depth_color, colormap);

    cv::imshow(windowName, depth_color);
}

int main() {
    float fps = 20.0f;

    // 1. Create pipeline
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(!device->isNeuralDepthSupported()) {
        std::cout << "Exiting NeuralAssistedStereo example: device doesn't support NeuralDepth.\n";
        return 0;
    }

    // 2. Define nodes
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto monoRightOut = monoRight->requestFullResolutionOutput();
    auto monoLeftOut = monoLeft->requestFullResolutionOutput();

    auto neuralAssistedStereo = pipeline.create<dai::node::NeuralAssistedStereo>()->build(*monoLeftOut, *monoRightOut);

    // 6. Get output queue
    auto disparityQueue = neuralAssistedStereo->disparity.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto disparityPacket = disparityQueue->get<dai::ImgFrame>();
        showDepth(disparityPacket->getCvFrame(), "Disparity View", 100, 6000, cv::COLORMAP_TURBO, false);
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
    return 0;
}

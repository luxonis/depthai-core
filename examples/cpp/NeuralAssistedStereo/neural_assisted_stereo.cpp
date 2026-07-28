#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/NeuralAssistedStereo.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr float FPS = 20.0f;
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
    auto depthQueue = neuralAssistedStereo->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depthPacket = depthQueue->get<dai::ImgFrame>();
        cv::imshow("Depth", dai::utility::colorizeDepthFrame(*depthPacket, 500.0f, 12000.0f, cv::COLORMAP_TURBO, true).getCvFrame());
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }
    return 0;
}

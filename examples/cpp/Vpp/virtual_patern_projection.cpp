#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

/**
  ┌──────┐       ┌───────────────┐ ----------------left------------------------------> ┌─────┐
  | Left | ----> |               |                                                     |     |
  └──────┘       |               | ----------------right-----------------------------> |     | -left--> ┌────────┐
                 | Rectification |                                                     | Vpp |          | Stereo | --depth->
  ┌───────┐      |               | --left_low_res---> ┌──────────────┐ --disparity---> |     |          |        |
  | Right | ---> |               |                    | NeuralStereo |                 |     | -right-> └────────┘
  └───────┘      └───────────────┘ --right_low_res--> └──────────────┘ --confidence--> └─────┘
**/

int main() {
    int fps = 20;
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(!device->isNeuralDepthSupported()) {
        std::cout << "Exiting Vpp example: device doesn't support NeuralDepth.\n";
        return 0;
    }

    // Left / Right cameras
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();

    // Rectification node
    auto rectification = pipeline.create<dai::node::Rectification>();
    monoLeftOut->link(rectification->input1);
    monoRightOut->link(rectification->input2);

    // Neural Depth builder
    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>()->build(*monoLeftOut, *monoRightOut, dai::DeviceModelZoo::NEURAL_DEPTH_NANO);

    // VPP node builder
    auto vpp = pipeline.create<dai::node::Vpp>()->build(rectification->output1, rectification->output2, neuralDepth->disparity, neuralDepth->confidence);

    // Set initial parameters
    vpp->initialConfig->blending = 0.5f;
    vpp->initialConfig->maxPatchSize = 2;
    vpp->initialConfig->patchColoringType = dai::VppConfig::PatchColoringType::RANDOM;
    vpp->initialConfig->uniformPatch = true;
    vpp->initialConfig->maxFPS = static_cast<float>(fps);

    auto& injectionParameters = vpp->initialConfig->injectionParameters;
    injectionParameters.textureThreshold = 3.0f;
    injectionParameters.useInjection = true;

    // Stereo depth node
    auto stereoNode = pipeline.create<dai::node::StereoDepth>();
    stereoNode->setRectification(false);

    vpp->leftOut.link(stereoNode->left);
    vpp->rightOut.link(stereoNode->right);

    // Output queues
    auto vppLeftQueue = vpp->leftOut.createOutputQueue();
    auto vppRightQueue = vpp->rightOut.createOutputQueue();
    auto depthQueue = stereoNode->depth.createOutputQueue();

    std::cout << "Pipeline started successfully" << std::endl;

    pipeline.start();

    while(true) {
        auto vppLeftFrame = vppLeftQueue->get<dai::ImgFrame>();
        auto vppRightFrame = vppRightQueue->get<dai::ImgFrame>();
        auto depthFrame = depthQueue->get<dai::ImgFrame>();

        cv::imshow("vppLeft", vppLeftFrame->getCvFrame());
        cv::imshow("vppRight", vppRightFrame->getCvFrame());

        cv::imshow("Depth", dai::utility::colorizeDepthFrame(*depthFrame, 500.0f, 12000.0f, cv::COLORMAP_TURBO, true).getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}

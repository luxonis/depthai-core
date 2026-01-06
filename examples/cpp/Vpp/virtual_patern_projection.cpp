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

// Nicely visualize a depth map.
void showDepth(const cv::Mat& depth_frame_in,
               const std::string& window_name = "Depth",
               int min_distance = 500,
               int max_distance = 5000,
               int colormap = cv::COLORMAP_TURBO,
               bool use_log = false) {
    cv::Mat depth_frame = depth_frame_in.clone();

    cv::Mat float_frame;
    depth_frame.convertTo(float_frame, CV_32FC1);

    // # Optionally apply log scaling
    if(use_log) {
        // depth_frame = np.log(depth_frame + 1)
        cv::log(float_frame + 1, float_frame);
    }

    cv::Mat upper_clamped;
    cv::min(float_frame, max_distance, upper_clamped);

    cv::Mat clipped_frame;
    cv::max(upper_clamped, min_distance, clipped_frame);

    double alpha = 255.0 / max_distance;
    clipped_frame.convertTo(clipped_frame, CV_8U, alpha);

    cv::Mat depth_color;
    cv::applyColorMap(clipped_frame, depth_color, colormap);

    cv::imshow(window_name, depth_color);
}

int main() {
    int fps = 20;
    dai::Pipeline pipeline;

    // Left Right cameras
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();

    // Rectification node
    auto rectification = pipeline.create<dai::node::Rectification>();
    monoLeftOut->link(rectification->input1);
    monoRightOut->link(rectification->input2);

    // Neural Depth Builder
    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>()->build(*monoLeftOut, *monoRightOut, dai::DeviceModelZoo::NEURAL_DEPTH_NANO);

    // Vpp node Builder
    auto vpp = pipeline.create<dai::node::Vpp>()->build(rectification->output1, rectification->output2, neuralDepth->disparity, neuralDepth->confidence);

    // Set initial parameters
    vpp->initialConfig->blending = 0.5f;
    vpp->initialConfig->maxPatchSize = 2;
    vpp->initialConfig->patchColoringType = dai::VppConfig::PatchColoringType::RANDOM;
    vpp->initialConfig->uniformPatch = true;
    vpp->initialConfig->maxFPS = (float)fps;

    auto& injectionParams = vpp->initialConfig->injectionParameters;
    injectionParams.textureThreshold = 3.0f;
    injectionParams.useInjection = true;

    // Stereo node
    auto stereoNode = pipeline.create<dai::node::StereoDepth>();
    stereoNode->setRectification(false);
    vpp->leftOut.link(stereoNode->left);
    vpp->rightOut.link(stereoNode->right);

    // Create output queues
    auto vppOutputLeft = vpp->leftOut.createOutputQueue();
    auto vppOutputRight = vpp->rightOut.createOutputQueue();
    auto depthQueue = stereoNode->depth.createOutputQueue();

    std::cout << "Pipeline started successfully" << std::endl;

    pipeline.start();
    while(true) {
        auto vppOutLeft = vppOutputLeft->get<dai::ImgFrame>();
        auto vppOutRight = vppOutputRight->get<dai::ImgFrame>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        cv::imshow("vpp_left", vppOutLeft->getCvFrame());
        cv::imshow("vpp_right", vppOutRight->getCvFrame());

        showDepth(depth->getCvFrame());

        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}

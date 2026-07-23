#include <csignal>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
static std::atomic<bool> extended_disparity{false};
// Better accuracy for longer distance, fractional disparity 32-levels:
static std::atomic<bool> subpixel{false};
// Better handling for occlusions:
static std::atomic<bool> lr_check{true};
static std::atomic<bool> quitEvent{false};

void signalHandler(int) {
    quitEvent = true;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto depth = pipeline.create<dai::node::StereoDepth>();

    // Properties
    auto* lout = monoLeft->requestOutput({640, 400});
    auto* rout = monoRight->requestOutput({640, 400});

    // Create a node that will produce the depth map.
    depth->build(*lout, *rout, dai::node::StereoDepth::PresetMode::DEFAULT);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig->setMedianFilter(dai::StereoDepthConfig::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(lr_check);
    depth->setExtendedDisparity(extended_disparity);
    depth->setSubpixel(subpixel);

    auto q = depth->depth.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning() && !quitEvent) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto frame = dai::utility::colorizeDepthFrame(*inDepth, 500.0f, 12000.0f, cv::COLORMAP_JET, true).getCvFrame();
        cv::imshow("depth", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    pipeline.stop();
    pipeline.wait();

    return 0;
}

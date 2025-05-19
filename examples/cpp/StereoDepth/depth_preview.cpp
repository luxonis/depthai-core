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

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto depth = pipeline.create<dai::node::StereoDepth>();

    // Properties
    auto* lout = monoLeft->requestOutput({640, 400});
    auto* rout = monoRight->requestOutput({640, 400});

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth->build(*lout, *rout, dai::node::StereoDepth::PresetMode::DEFAULT);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig->setMedianFilter(dai::StereoDepthConfig::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(lr_check);
    depth->setExtendedDisparity(extended_disparity);
    depth->setSubpixel(subpixel);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q = depth->disparity.createOutputQueue();
    auto qleft = lout->createOutputQueue();

    pipeline.start();

    while(true) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto inLeft = qleft->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        // Normalization for better visualization
        frame.convertTo(frame, CV_8UC1, 255 / depth->initialConfig->getMaxDisparity());

        std::cout << "Left type: " << inLeft->fb.str() << std::endl;

        cv::imshow("disparity", frame);

        // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
        cv::imshow("disparity_color", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    pipeline.stop();
    return 0;
}

#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;
    // TODO - split this example into two separate examples
    bool withDepth = true;

    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;
    auto xoutDisp = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    // monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    // monoRight->setFps(5.0);

    bool outputDepth = false;
    bool outputRectified = true;
    bool lrcheck = true;
    bool extended = false;
    bool subpixel = false;

    int maxDisp = 96;
    if(extended) maxDisp *= 2;
    if(subpixel) maxDisp *= 32;  // 5 bits fractional disparity

    if(withDepth) {
        // StereoDepth
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        // stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
        // stereo->setInputResolution(1280, 720);
        // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
        stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
        stereo->disparity.link(xoutDisp->input);
        if(outputDepth) {
            stereo->depth.link(xoutDepth->input);
        }

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // Connect and start the pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 8, false);
    auto rightQueue = device.getOutputQueue("right", 8, false);
    auto dispQueue = withDepth ? device.getOutputQueue("disparity", 8, false) : nullptr;
    auto depthQueue = withDepth ? device.getOutputQueue("depth", 8, false) : nullptr;
    auto rectifLeftQueue = withDepth ? device.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? device.getOutputQueue("rectified_right", 8, false) : nullptr;

    while(1) {
        auto left = leftQueue->get<dai::ImgFrame>();
        cv::imshow("left", cv::Mat(left->getHeight(), left->getWidth(), CV_8UC1, left->getData().data()));
        auto right = rightQueue->get<dai::ImgFrame>();
        cv::imshow("right", cv::Mat(right->getHeight(), right->getWidth(), CV_8UC1, right->getData().data()));

        if(withDepth) {
            // Note: in some configurations (if depth is enabled), disparity may output garbage data
            auto disparity = dispQueue->get<dai::ImgFrame>();
            cv::Mat disp(disparity->getHeight(), disparity->getWidth(), subpixel ? CV_16UC1 : CV_8UC1, disparity->getData().data());
            disp.convertTo(disp, CV_8UC1, 255.0 / maxDisp);  // Extend disparity range
            cv::imshow("disparity", disp);
            cv::Mat disp_color;
            cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);
            cv::imshow("disparity_color", disp_color);

            if(outputDepth) {
                auto depth = depthQueue->get<dai::ImgFrame>();
                cv::imshow("depth", cv::Mat(depth->getHeight(), depth->getWidth(), CV_16UC1, depth->getData().data()));
            }

            if(outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                cv::Mat rectifiedLeftFrame = rectifL->getFrame();
                // cv::flip(rectifiedLeftFrame, rectifiedLeftFrame, 1);
                cv::imshow("rectified_left", rectifiedLeftFrame);

                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                cv::Mat rectifiedRightFrame = rectifR->getFrame();

                // cv::flip(rectifiedRightFrame, rectifiedRightFrame, 1);
                cv::imshow("rectified_right", rectifiedRightFrame);
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}

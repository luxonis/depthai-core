

#include <iostream>
#include <cstdio>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(){
    using namespace std;

    // TODO - split this example into two separate examples
    bool withDepth = true;

    dai::Pipeline p;

    auto monoLeft  = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto xoutLeft  = p.create<dai::node::XLinkOut>();
    auto xoutRight = p.create<dai::node::XLinkOut>();
    auto stereo    = withDepth ? p.create<dai::node::StereoDepth>() : nullptr;
    auto xoutDisp  = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();
    auto xoutRectifL = p.create<dai::node::XLinkOut>();
    auto xoutRectifR = p.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if (withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    //monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    //monoRight->setFps(5.0);

    bool outputDepth = false;
    bool outputRectified = true;
    bool lrcheck  = true;
    bool extended = false;
    bool subpixel = true;

    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    if (withDepth) {
        // StereoDepth
        stereo->setOutputDepth(outputDepth);
        stereo->setOutputRectified(outputRectified);
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        //stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
        //stereo->setInputResolution(1280, 720);
        // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
        //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        if(outputRectified)
        {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
        stereo->disparity.link(xoutDisp->input);
        stereo->depth.link(xoutDepth->input);

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // CONNECT TO DEVICE
    dai::Device d(p);
    d.startPipeline();

    auto leftQueue = d.getOutputQueue("left", 8, false);
    auto rightQueue = d.getOutputQueue("right", 8, false);
    auto dispQueue = withDepth ? d.getOutputQueue("disparity", 8, false) : nullptr;
    auto depthQueue = withDepth ? d.getOutputQueue("depth", 8, false) : nullptr;
    auto rectifLeftQueue = withDepth ? d.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? d.getOutputQueue("rectified_right", 8, false) : nullptr;

    while (1) {
        auto t1 = std::chrono::steady_clock::now();
        auto left = leftQueue->get<dai::ImgFrame>();
        auto t2 = std::chrono::steady_clock::now();
        cv::imshow("left", cv::Mat(left->getHeight(), left->getWidth(), CV_8UC1, left->getData().data()));
        auto t3 = std::chrono::steady_clock::now();
        auto right = rightQueue->get<dai::ImgFrame>();
        auto t4 = std::chrono::steady_clock::now();
        cv::imshow("right", cv::Mat(right->getHeight(), right->getWidth(), CV_8UC1, right->getData().data()));
        auto t5 = std::chrono::steady_clock::now();

        if (withDepth) {
            // Note: in some configurations (if depth is enabled), disparity may output garbage data
            auto disparity = dispQueue->get<dai::ImgFrame>();
            cv::Mat disp(disparity->getHeight(), disparity->getWidth(),
                    subpixel ? CV_16UC1 : CV_8UC1, disparity->getData().data());
            disp.convertTo(disp, CV_8UC1, 255.0 / maxDisp); // Extend disparity range
            cv::imshow("disparity", disp);
            cv::Mat disp_color;
            cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);
            cv::imshow("disparity_color", disp_color);

            if (outputDepth) {
                auto depth = depthQueue->get<dai::ImgFrame>();
                cv::imshow("depth", cv::Mat(depth->getHeight(), depth->getWidth(),
                        CV_16UC1, depth->getData().data()));
            }

            if (outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                cv::imshow("rectified_left", cv::Mat(rectifL->getHeight(), rectifL->getWidth(),
                        CV_8UC1, rectifL->getData().data()));
                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                cv::imshow("rectified_right", cv::Mat(rectifR->getHeight(), rectifR->getWidth(),
                        CV_8UC1, rectifR->getData().data()));
            }
        }

        int ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        int ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count();
        int ms3 = std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count();
        int ms4 = std::chrono::duration_cast<std::chrono::milliseconds>(t5-t4).count();
        int loop = std::chrono::duration_cast<std::chrono::milliseconds>(t5-t1).count();

        std::cout << ms1 << " " << ms2 << " " << ms3 << " " << ms4 << " loop: " << loop << std::endl;
        int key = cv::waitKey(1);
        if (key == 'q'){
            return 0;
        }

    }
}


#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static std::atomic<bool> outputDepth{false};
static std::atomic<bool> outputRectified{true};
static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};

int main(int argc, char** argv) {
    using namespace std;

    // TODO: Add all other arguments
    string test1, test2;
    if(argc > 2) {
        test1 = string(argv[1]);
        test2 = string(argv[2]);
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDisp = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutDisp->setStreamName("disparity");
    xoutDepth->setStreamName("depth");
    xoutRectifL->setStreamName("rectified_left");
    xoutRectifR->setStreamName("rectified_right");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(200);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::MEDIAN_OFF);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);
    stereo->disparity.link(xoutDisp->input);

    if(outputRectified) {
        stereo->rectifiedLeft.link(xoutRectifL->input);
        stereo->rectifiedRight.link(xoutRectifR->input);
    }

    if(outputDepth) {
        stereo->depth.link(xoutDepth->input);
    }

    // Get mesh data
    auto calibData = dai::Device().readCalibration();

    int resolution[] = {1280, 720};
    auto M1 = calibData.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, resolution[0], resolution[1]);
    auto d1 = calibData.getDistortionCoefficients(dai::CameraBoardSocket::LEFT);
    auto R1 = calibData.getStereoLeftRectificationRotation();
    auto M2 = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, resolution[0], resolution[1]);
    auto d2 = calibData.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);
    auto R2 = calibData.getStereoRightRectificationRotation();

    // FIXME: (-215:Assertion failed) 0 <= i && i < (int)vv.size() in function 'getMat_'
    cv::Mat map_x_l, map_y_l;
    cv::initUndistortRectifyMap(M1, d1, R1, M1, cv::Size(resolution[0], resolution[1]), CV_32FC1, map_x_l, map_y_l);

    cv::Mat map_x_r, map_y_r;
    cv::initUndistortRectifyMap(M2, d2, R2, M2, cv::Size(resolution[0], resolution[1]), CV_32FC1, map_x_r, map_y_r);

    // // stereo->loadMeshData(map_x_l, map_y_l);

    // int meshCellSize = 16;
    // vector<vector<uint>> mesh_left, mesh_right;

    // for(int y = 0; y < map_x_l.rows + 1; y++) {
    //     if(y % meshCellSize == 0) {
    //         vector<uint> row_left, row_right;
    //         for(int x = 0; x < map_x_l.cols + 1; x++) {
    //             if(x % meshCellSize == 0) {
    //                 if(y == map_x_l.rows and x == map_x_l.cols) {
    //                     row_left.push_back(map_y_l.at<int>(y - 1, x - 1));
    //                     row_left.push_back(map_x_l.at<int>(y - 1, x - 1));
    //                     row_right.push_back(map_y_r.at<int>(y - 1, x - 1));
    //                     row_right.push_back(map_x_r.at<int>(y - 1, x - 1));
    //                 } else if(y == map_x_l.rows) {
    //                     row_left.push_back(map_y_l.at<int>(y - 1, x));
    //                     row_left.push_back(map_x_l.at<int>(y - 1, x));
    //                     row_right.push_back(map_y_r.at<int>(y - 1, x));
    //                     row_right.push_back(map_x_r.at<int>(y - 1, x));
    //                 } else if(x == map_x_l.cols) {
    //                     row_left.push_back(map_y_l.at<int>(y, x - 1));
    //                     row_left.push_back(map_x_l.at<int>(y, x - 1));
    //                     row_right.push_back(map_y_r.at<int>(y, x - 1));
    //                     row_right.push_back(map_x_r.at<int>(y, x - 1));
    //                 } else {
    //                     row_left.push_back(map_y_l.at<int>(y, x));
    //                     row_left.push_back(map_x_l.at<int>(y, x));
    //                     row_right.push_back(map_y_r.at<int>(y, x));
    //                     row_right.push_back(map_x_r.at<int>(y, x));
    //                 }
    //             }
    //         }
    //         if((map_x_l.rows % meshCellSize) % 2 != 0) {
    //             row_left.push_back(0);
    //             row_left.push_back(0);
    //             row_right.push_back(0);
    //             row_right.push_back(0);
    //         }

    //         mesh_left.push_back(row_left);
    //         mesh_right.push_back(row_right);
    //     }
    // }

    // TODO: Convert mesh_left, mesh_right to vectors of bytes
    // stereo->loadMeshData(mesh_left, mesh_right);

    // TODO: Add option for saving mesh to file

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 8, false);
    auto rightQueue = device.getOutputQueue("right", 8, false);
    auto dispQueue = device.getOutputQueue("disparity", 8, false);
    auto depthQueue = device.getOutputQueue("depth", 8, false);
    auto rectifLeftQueue = device.getOutputQueue("rectified_left", 8, false);
    auto rectifRightQueue = device.getOutputQueue("rectified_right", 8, false);

    // Disparity range is used for normalization
    float disparityMultiplier = 255 / stereo->getMaxDisparity();

    while(true) {
        auto left = leftQueue->get<dai::ImgFrame>();
        cv::imshow("left", left->getFrame());
        auto right = rightQueue->get<dai::ImgFrame>();
        cv::imshow("right", right->getFrame());

        // Note: in some configurations (if depth is enabled), disparity may output garbage data
        auto disparity = dispQueue->get<dai::ImgFrame>();
        cv::Mat disp(disparity->getCvFrame());
        disp.convertTo(disp, CV_8UC1, disparityMultiplier);  // Extend disparity range
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
            // cv::flip(rectifiedLeftFrame, rectifiedLeftFrame, 1);
            cv::imshow("rectified_left", rectifL->getFrame());

            auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
            // cv::flip(rectifiedRightFrame, rectifiedRightFrame, 1);
            cv::imshow("rectified_right", rectifR->getFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
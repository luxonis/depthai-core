#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Helper function to draw rotated rectangle
void drawRotatedRectangle(cv::Mat& frame, const cv::Point2f& center, const cv::Size2f& size, float angle, const cv::Scalar& color, int thickness = 2) {
    // Create a rotated rectangle
    cv::RotatedRect rect(center, size, angle);

    // Get the four vertices of the rotated rectangle
    cv::Point2f vertices[4];
    rect.points(vertices);

    // Convert vertices to integer points
    std::vector<cv::Point> points;
    for(int i = 0; i < 4; i++) {
        points.push_back(cv::Point(static_cast<int>(vertices[i].x), static_cast<int>(vertices[i].y)));
    }

    // Draw the rectangle
    cv::polylines(frame, points, true, color, thickness);
}

// Helper function to process depth frame
cv::Mat processDepthFrame(const cv::Mat& depthFrame) {
    cv::Mat depth_downscaled;
    cv::resize(depthFrame, depth_downscaled, cv::Size(), 0.25, 0.25);

    double min_depth = 0;
    if(!cv::countNonZero(depth_downscaled == 0)) {
        std::vector<uint16_t> nonZeroDepth;
        nonZeroDepth.reserve(depth_downscaled.rows * depth_downscaled.cols);

        for(int i = 0; i < depth_downscaled.rows; i++) {
            for(int j = 0; j < depth_downscaled.cols; j++) {
                uint16_t depth = depth_downscaled.at<uint16_t>(i, j);
                if(depth > 0) nonZeroDepth.push_back(depth);
            }
        }

        if(!nonZeroDepth.empty()) {
            std::sort(nonZeroDepth.begin(), nonZeroDepth.end());
            min_depth = nonZeroDepth[static_cast<int>(nonZeroDepth.size() * 0.01)];  // 1st percentile
        }
    }

    std::vector<uint16_t> allDepth;
    allDepth.reserve(depth_downscaled.rows * depth_downscaled.cols);
    for(int i = 0; i < depth_downscaled.rows; i++) {
        for(int j = 0; j < depth_downscaled.cols; j++) {
            allDepth.push_back(depth_downscaled.at<uint16_t>(i, j));
        }
    }
    std::sort(allDepth.begin(), allDepth.end());
    double max_depth = allDepth[static_cast<int>(allDepth.size() * 0.99)];  // 99th percentile

    // Normalize and colorize
    cv::Mat normalized;
    cv::normalize(depthFrame, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1, depthFrame > min_depth);
    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_HOT);
    return colorized;
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create and configure nodes
    auto color = pipeline.create<dai::node::Camera>();
    color->build(dai::CameraBoardSocket::CAM_A);

    auto monoLeft = pipeline.create<dai::node::Camera>();
    monoLeft->build(dai::CameraBoardSocket::CAM_B);

    auto monoRight = pipeline.create<dai::node::Camera>();
    monoRight->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // Configure stereo node
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
    // Uncomment to align depth to RGB
    // stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    // stereo->setOutputSize(640, 400);

    // Configure outputs
    auto colorCamOut = color->requestOutput(std::make_pair(640, 480));
    auto monoLeftOut = monoLeft->requestOutput(std::make_pair(640, 480));
    auto monoRightOut = monoRight->requestOutput(std::make_pair(640, 480));

    // Link mono cameras to stereo
    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    // Create output queues
    auto colorOut = colorCamOut->createOutputQueue();
    auto rightOut = monoRightOut->createOutputQueue();
    auto stereoOut = stereo->depth.createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto colorFrame = colorOut->get<dai::ImgFrame>();
        auto stereoFrame = stereoOut->get<dai::ImgFrame>();

        if(colorFrame == nullptr || stereoFrame == nullptr) continue;

        // Validate transformations
        if(!colorFrame->validateTransformations() || !stereoFrame->validateTransformations()) {
            std::cerr << "Invalid transformations!" << std::endl;
            continue;
        }

        // Get frames
        cv::Mat clr = colorFrame->getCvFrame();
        cv::Mat depth = processDepthFrame(stereoFrame->getCvFrame());

        // Create and remap rectangle
        dai::RotatedRect rect(dai::Point2f(300, 200), dai::Size2f(200, 100), 10);
        auto remappedRect = colorFrame->transformation.remapRectTo(stereoFrame->transformation, rect);

        // Print rectangle information
        std::cout << "Original rect x: " << rect.center.x << " y: " << rect.center.y << " width: " << rect.size.width << " height: " << rect.size.height
                  << " angle: " << rect.angle << std::endl;
        std::cout << "Remapped rect x: " << remappedRect.center.x << " y: " << remappedRect.center.y << " width: " << remappedRect.size.width
                  << " height: " << remappedRect.size.height << " angle: " << remappedRect.angle << std::endl;

        // Draw rectangles
        drawRotatedRectangle(clr, cv::Point2f(rect.center.x, rect.center.y), cv::Size2f(rect.size.width, rect.size.height), rect.angle, cv::Scalar(255, 0, 0));

        drawRotatedRectangle(depth,
                             cv::Point2f(remappedRect.center.x, remappedRect.center.y),
                             cv::Size2f(remappedRect.size.width, remappedRect.size.height),
                             remappedRect.angle,
                             cv::Scalar(255, 0, 0));

        // Show frames
        cv::imshow("color", clr);
        cv::imshow("depth", depth);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
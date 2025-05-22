#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Helper function to draw rotated rectangle
void draw_rotated_rectangle(cv::Mat& frame, const cv::Point2f& center, const cv::Size2f& size, float angle, const cv::Scalar& color, int thickness = 2) {
    // Create a rotated rectangle
    cv::RotatedRect rect(center, size, angle);

    // Get the four vertices of the rotated rectangle
    cv::Point2f vertices[4];
    rect.points(vertices);

    // Convert to integer points for drawing
    std::vector<cv::Point> points;
    for(int i = 0; i < 4; i++) {
        points.push_back(cv::Point(static_cast<int>(vertices[i].x), static_cast<int>(vertices[i].y)));
    }

    // Draw the rectangle on the frame
    cv::polylines(frame, std::vector<std::vector<cv::Point>>{points}, true, color, thickness);
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Create nodes
        auto cam = pipeline.create<dai::node::Camera>();
        cam->build();

        auto manip1 = pipeline.create<dai::node::ImageManip>();
        auto manip2 = pipeline.create<dai::node::ImageManip>();

        // Configure camera output
        auto camOut = cam->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::LETTERBOX, 20, 20);

        // Configure image manipulators
        manip1->initialConfig->addRotateDeg(90);
        manip1->initialConfig->setOutputSize(200, 320);

        manip2->initialConfig->addRotateDeg(90);
        manip2->initialConfig->setOutputSize(320, 200);
        manip2->setRunOnHost(true);

        // Linking
        camOut->link(manip1->inputImage);
        manip1->out.link(manip2->inputImage);

        // Create output queues
        auto outQcam = camOut->createOutputQueue();
        auto outQ1 = manip1->out.createOutputQueue();
        auto outQ2 = manip2->out.createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Main loop
        while(!quitEvent) {
            // Get frames
            auto camFrame = outQcam->get<dai::ImgFrame>();
            auto manip1Frame = outQ1->get<dai::ImgFrame>();
            auto manip2Frame = outQ2->get<dai::ImgFrame>();

            // Convert to OpenCV format
            cv::Mat camCv = camFrame->getCvFrame();
            cv::Mat manip1Cv = manip1Frame->getCvFrame();
            cv::Mat manip2Cv = manip2Frame->getCvFrame();

            // Create and remap rectangles
            dai::RotatedRect rect2(dai::Rect(dai::Point2f(100, 100), dai::Point2f(200, 150)), 0);
            auto rect1 = manip2Frame->transformation.remapRectTo(manip1Frame->transformation, rect2);
            auto rectcam = manip1Frame->transformation.remapRectTo(camFrame->transformation, rect1);

            // Draw rectangles
            draw_rotated_rectangle(
                manip2Cv, cv::Point2f(rect2.center.x, rect2.center.y), cv::Size2f(rect2.size.width, rect2.size.height), rect2.angle, cv::Scalar(255, 0, 0));

            draw_rotated_rectangle(
                manip1Cv, cv::Point2f(rect1.center.x, rect1.center.y), cv::Size2f(rect1.size.width, rect1.size.height), rect1.angle, cv::Scalar(255, 0, 0));

            draw_rotated_rectangle(camCv,
                                   cv::Point2f(rectcam.center.x, rectcam.center.y),
                                   cv::Size2f(rectcam.size.width, rectcam.size.height),
                                   rectcam.angle,
                                   cv::Scalar(255, 0, 0));

            // Display frames
            cv::imshow("cam", camCv);
            cv::imshow("manip1", manip1Cv);
            cv::imshow("manip2", manip2Cv);

            // Check for quit key
            if(cv::waitKey(1) == 'q') {
                break;
            }
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
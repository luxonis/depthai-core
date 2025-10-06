#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Custom threaded host node for camera capture
class HostCamera : public dai::node::CustomThreadedNode<HostCamera> {
   public:
    HostCamera() {
        output = std::shared_ptr<dai::Node::Output>(new dai::Node::Output(*this, {"output", DEFAULT_GROUP, {{{dai::DatatypeEnum::ImgFrame, false}}}}));
    }

    void run() override {
        std::cout << "HostCamera running" << std::endl;
        // Create a VideoCapture object
        cv::VideoCapture cap(0);
        if(!cap.isOpened()) {
            std::cerr << "Error: Couldn't open host camera" << std::endl;
            stopPipeline();
            return;
        }

        while(mainLoop()) {
            // Read the frame from the camera
            cv::Mat frame;
            if(!cap.read(frame)) {
                break;
            }

            // Create an ImgFrame message
            auto imgFrame = std::make_shared<dai::ImgFrame>();

            std::vector<uchar> buffer(frame.data, frame.data + frame.total() * frame.elemSize());
            imgFrame->setData(buffer);
            imgFrame->setWidth(frame.cols);
            imgFrame->setHeight(frame.rows);
            imgFrame->setType(dai::ImgFrame::Type::BGR888i);

            // Send the message
            output->send(imgFrame);

            // Wait for the next frame
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        cap.release();
    }

    std::shared_ptr<dai::Node::Output> output;
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline without implicit device
    dai::Pipeline pipeline(false);

    // Create host camera node
    auto hostCamera = pipeline.create<HostCamera>();
    auto camQueue = hostCamera->output->createOutputQueue();

    // Start pipeline
    pipeline.start();
    std::cout << "Host camera started. Press 'q' to quit." << std::endl;

    while(pipeline.isRunning() && !quitEvent) {
        auto image = camQueue->get<dai::ImgFrame>();
        if(image == nullptr) continue;

        cv::imshow("HostCamera", image->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q') {
            pipeline.stop();
            break;
        }
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    return 0;
}

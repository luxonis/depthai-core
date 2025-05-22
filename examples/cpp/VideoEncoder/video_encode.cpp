#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Custom host node for saving video data
class VideoSaver : public dai::node::CustomNode<VideoSaver> {
   public:
    VideoSaver() : fileHandle("video.encoded", std::ios::binary) {
        if(!fileHandle.is_open()) {
            throw std::runtime_error("Could not open video.encoded for writing");
        }
    }

    ~VideoSaver() {
        if(fileHandle.is_open()) {
            fileHandle.close();
        }
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> message) override {
        if(!fileHandle.is_open()) return nullptr;

        // Get raw data and write to file
        auto frame = message->get<dai::EncodedFrame>("data");
        unsigned char* frameData = frame->getData().data();
        size_t frameSize = frame->getData().size();
        std::cout << "Storing frame of size: " << frameSize << std::endl;
        fileHandle.write(reinterpret_cast<const char*>(frameData), frameSize);

        // Don't send anything back
        return nullptr;
    }

   private:
    std::ofstream fileHandle;
};

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto output = camRgb->requestOutput(std::make_pair(1920, 1440), dai::ImgFrame::Type::NV12);
    auto outputQueue = output->createOutputQueue();

    // Create video encoder node
    auto encoded = pipeline.create<dai::node::VideoEncoder>();
    encoded->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    output->link(encoded->input);

    // Create video saver node
    auto saver = pipeline.create<VideoSaver>();
    encoded->out.link(saver->inputs["data"]);

    // Start pipeline
    pipeline.start();
    std::cout << "Started to save video to video.encoded" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    auto timeStart = std::chrono::steady_clock::now();

    while(pipeline.isRunning() && !quitEvent) {
        auto frame = outputQueue->get<dai::ImgFrame>();
        if(frame == nullptr) continue;

        cv::imshow("video", frame->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    std::cout << "To view the encoded data, convert the stream file (.encoded) into a video file (.mp4) using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate 30 -i video.encoded -c copy video.mp4" << std::endl;

    return 0;
}
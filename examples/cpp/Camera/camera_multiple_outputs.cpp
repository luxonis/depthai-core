#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        if(frameTimes.size() > 100) {
            frameTimes.erase(frameTimes.begin());
        }
    }

    float getFps() {
        if(frameTimes.size() <= 1) return 0.0f;
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(frameTimes.back() - frameTimes.front()).count();
        return (frameTimes.size() - 1) * 1000.0f / duration;
    }

   private:
    std::vector<std::chrono::steady_clock::time_point> frameTimes;
};

void exitUsage() {
    std::cout << "WRONG USAGE! correct usage example:\n"
              << "./camera_multiple_outputs 640 480 0 30 CAM_A 300 300 0 30 CAM_A 300 300 1 30 CAM_A\n"
              << "where 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX\n"
              << "and 30 is FPS" << std::endl;
    exit(1);
}

int main(int argc, char* argv[]) {
    if(argc < 6 || (argc - 1) % 5 != 0) {
        exitUsage();
    }

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Parse arguments and create cameras
    std::map<dai::CameraBoardSocket, std::shared_ptr<dai::node::Camera>> cams;
    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    std::vector<FPSCounter> fpsCounters;

    for(int i = 1; i < argc; i += 5) {
        int width = std::stoi(argv[i]);
        int height = std::stoi(argv[i + 1]);
        int resizeMode = std::stoi(argv[i + 2]);
        float fps = std::stof(argv[i + 3]);
        std::string camArg = argv[i + 4];

        // Create capability
        auto cap = std::make_shared<dai::ImgFrameCapability>();
        cap->size.fixed(std::make_pair(width, height));

        // Set resize mode
        switch(resizeMode) {
            case 0:
                cap->resizeMode = dai::ImgResizeMode::CROP;
                break;
            case 1:
                cap->resizeMode = dai::ImgResizeMode::STRETCH;
                break;
            case 2:
                cap->resizeMode = dai::ImgResizeMode::LETTERBOX;
                break;
            default:
                exitUsage();
        }

        cap->fps.fixed(fps);

        // Parse camera socket
        dai::CameraBoardSocket socket;
        if(camArg == "CAM_A")
            socket = dai::CameraBoardSocket::CAM_A;
        else if(camArg == "CAM_B")
            socket = dai::CameraBoardSocket::CAM_B;
        else if(camArg == "CAM_C")
            socket = dai::CameraBoardSocket::CAM_C;
        else if(camArg == "CAM_D")
            socket = dai::CameraBoardSocket::CAM_D;
        else
            exitUsage();

        // Create camera if not exists
        if(cams.find(socket) == cams.end()) {
            cams[socket] = pipeline.create<dai::node::Camera>()->build(socket);
        }

        // Create output queue
        queues.push_back(cams[socket]->requestOutput(*cap, true)->createOutputQueue());
        fpsCounters.push_back(FPSCounter());
    }

    // Start pipeline
    pipeline.start();

    while(true) {
        for(size_t i = 0; i < queues.size(); i++) {
            auto videoIn = queues[i]->tryGet<dai::ImgFrame>();
            if(videoIn != nullptr) {
                fpsCounters[i].tick();
                std::cout << "frame " << videoIn->getWidth() << "x" << videoIn->getHeight() << " | " << videoIn->getSequenceNum()
                          << ": exposure=" << videoIn->getExposureTime().count()
                          << "us, timestamp: " << videoIn->getTimestampDevice().time_since_epoch().count() << std::endl;

                cv::Mat cvFrame = videoIn->getCvFrame();

                // Draw FPS
                cv::putText(cvFrame,
                            std::to_string(fpsCounters[i].getFps()).substr(0, 4) + " FPS",
                            cv::Point(2, 20),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(0, 255, 0));

                cv::imshow("video " + std::to_string(i), cvFrame);
            }
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
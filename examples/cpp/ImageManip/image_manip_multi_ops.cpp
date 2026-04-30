#include <atomic>
#include <csignal>
#include <memory>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/host/Display.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

int main(int argc, char** argv) {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    std::shared_ptr<dai::Device> device = nullptr;
    if(argc <= 1) {
        device = std::make_shared<dai::Device>();
    } else {
        device = std::make_shared<dai::Device>(argv[1]);
    }
    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto manip = pipeline.create<dai::node::ImageManip>();

    manip->setMaxOutputFrameSize(4000000);
    manip->initialConfig->setOutputSize(1280, 720, dai::ImageManipConfig::ResizeMode::LETTERBOX);
    manip->initialConfig->setBackgroundColor(100, 100, 100);
    manip->initialConfig->addRotateDeg(45);
    manip->initialConfig->addCrop(100, 100, 800, 600);
    manip->initialConfig->addFlipVertical();
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::RGB888p);

    auto* rgbOut = camRgb->requestOutput({1920, 1080});
    rgbOut->link(manip->inputImage);
    auto outputQueue = manip->out.createOutputQueue();
    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        auto imgFrame = outputQueue->get<dai::ImgFrame>();
        cv::imshow("Manipulated Frame", imgFrame->getCvFrame());
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();
    pipeline.wait();

    return 0;
}

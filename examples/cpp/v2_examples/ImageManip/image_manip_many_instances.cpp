#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

int main() {
    dai::DeviceInfo info("127.0.0.1");
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);

    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::ColorCamera>();

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    // camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    // camRgb->setVideoSize(1280, 720);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1920, 1080);

    std::vector<std::shared_ptr<dai::node::ImageManip>> resizeNodes;
    int newHeightCount = 1;
    int newNewHeightCount = 1;
    for(int index = 0; index < 3; ++index) {
        double height = 300.0 * (index + 1);
        if(height > 1080) {
            height = 170.0 * newHeightCount;
            if(height > 1080) {
                height = 160.0 * newNewHeightCount;
                ++newNewHeightCount;
            } else {
                ++newHeightCount;
            }
        }
        // const double height = 300.0;
        auto imageManip = pipeline.create<dai::node::ImageManip>();
        const double ratio = height / 1080.0;
        int width = std::ceil(1920.0 * ratio);
        width = (width % 2 == 1) ? width + 1 : width;
        std::cout << "Width was: " << width << "\n" << std::flush;
        imageManip->setMaxOutputFrameSize(2000 * 2000 * 3);
        // imageManip->initialConfig.setCropRect(0.4f, 0.4f, 0.6f, 0.6f);
        // imageManip->initialConfig.setResize(static_cast<int>(ratio * 1920.f), static_cast<int>(ratio * 1080.f));
        imageManip->initialConfig.setResize(width, height);
        // imageManip->initialConfig.setCropRect(0.4f, 0.4f, 0.6f, 0.6f);
        camRgb->video.link(imageManip->inputImage);
        resizeNodes.push_back(imageManip);
    }

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->setMaxOutputFrameSize(2000 * 2000 * 3);
    imageManip->initialConfig.setCropRect(0.4f, 0.4f, 0.6f, 0.6f);
    camRgb->video.link(imageManip->inputImage);
    resizeNodes.push_back(imageManip);

    /*
    auto imageManipCrop = pipeline.create<dai::node::ImageManip>();
    imageManipCrop->setMaxOutputFrameSize(2000 * 2000 * 3);
    const double cutDiff = (1920.0 - 1080.0) / 2.0 * ratio / height;
    std::cout << cutDiff << " | " << 1.0 - cutDiff << "\n" << std::flush;
    imageManipCrop->initialConfig.setCropRect(cutDiff, 0.0, 1.0 - cutDiff, 1.0);

    imageManip->out.link(imageManipCrop->inputImage);
    */

    auto video = camRgb->video.createOutputQueue();
    std::vector<std::shared_ptr<dai::MessageQueue>> resizedOutputs;
    for(auto imageManip : resizeNodes) {
        resizedOutputs.push_back(imageManip->out.createOutputQueue());
    }

    pipeline.start();

    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();
        // auto croppedIn = cropped->tryGet<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int index = 0;
        for(auto resized : resizedOutputs) {
            auto resizedIn = resized->get<dai::ImgFrame>();
            if(resizedIn) {
                cv::imshow("resized " + std::to_string(index), resizedIn->getCvFrame());
            }
            ++index;
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
            return 0;
        }
    }
    pipeline.stop();
    return 0;
}

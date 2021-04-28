#include <cstdio>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv)
{
    using namespace std::chrono;

    dai::Pipeline pipeline;

    // Define source and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutJpeg = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();

    xoutJpeg->setStreamName("jpeg");
    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    videoEnc->setDefaultProfilePreset(camRgb->getVideoSize(), camRgb->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking
    camRgb->video.link(xoutRgb->input);
    camRgb->video.link(videoEnc->input);
    videoEnc->bitstream.link(xoutJpeg->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device device(pipeline);
    // Start pipeline
    device.startPipeline();

    // Queues
    auto qRgb = device.getOutputQueue("rgb", 30, false);
    auto qJpeg = device.getOutputQueue("jpeg", 30, true);

    mkdir("06_data", 0777);

    while(true) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if (inRgb != NULL){
            cv::imshow("rgb", inRgb->getCvFrame());
        }

        auto encFrames = qJpeg->tryGetAll<dai::ImgFrame>();
        for(const auto& encFrame : encFrames){
            uint64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::stringstream videoStr;
            videoStr << "06_data/" << time << ".jpeg";
            auto videoFile = std::ofstream(videoStr.str(), std::ios::binary);
            videoFile.write((char*)encFrame->getData().data(), encFrame->getData().size());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q')
            return 0;
    }
    return 0;
}

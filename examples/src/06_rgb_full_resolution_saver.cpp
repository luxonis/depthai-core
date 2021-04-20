#include <cstdio>
#include <iostream>
#include <sys/stat.h>
#include "utility.hpp"
#include <chrono>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv)
{
    using namespace std::chrono;

    dai::Pipeline p;

    // Define a source - color camera
    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xoutJpeg = p.create<dai::node::XLinkOut>();
    auto xoutRgb = p.create<dai::node::XLinkOut>();
    auto videoEnc = p.create<dai::node::VideoEncoder>();

    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    xoutJpeg->setStreamName("jpeg");
    xoutRgb->setStreamName("rgb");
    videoEnc->setDefaultProfilePreset(3840, 2160, 30, dai::VideoEncoderProperties::Profile::MJPEG);

    // Create outputs
    colorCam->video.link(xoutRgb->input);
    colorCam->video.link(videoEnc->input);
    videoEnc->bitstream.link(xoutJpeg->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(p);
    // Start pipeline
    d.startPipeline();

    // Queues
    auto qRgb = d.getOutputQueue("rgb", 30, false);
    auto qJpeg = d.getOutputQueue("jpeg", 30, true);

    mkdir("06_data", 0777);

    while(true) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if (inRgb != NULL){
            cv::Mat frame = inRgb->getCvFrame();
            cv::imshow("rgb", frame);
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
        if(key == 'q')
            return 0;
    }

    return 0;
}

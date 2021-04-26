#include <csignal>
#include <cstdio>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

    // Keyboard interrupt (Ctrl + C) detected
static std::atomic<bool> alive{true};
static void sigintHandler(int signum) {
    alive = false;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::signal(SIGINT, &sigintHandler);

    dai::Pipeline p;

    // Define a source - color camera and 2 mono camera
    auto colorCam = p.create<dai::node::ColorCamera>();
    auto monoCam = p.create<dai::node::MonoCamera>();
    auto monoCam2 = p.create<dai::node::MonoCamera>();
    auto ve1 = p.create<dai::node::VideoEncoder>();
    auto ve2 = p.create<dai::node::VideoEncoder>();
    auto ve3 = p.create<dai::node::VideoEncoder>();

    auto ve1Out = p.create<dai::node::XLinkOut>();
    auto ve2Out = p.create<dai::node::XLinkOut>();
    auto ve3Out = p.create<dai::node::XLinkOut>();

    // Properties
    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    monoCam->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoCam2->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    ve1Out->setStreamName("ve1Out");
    ve2Out->setStreamName("ve2Out");
    ve3Out->setStreamName("ve3Out");
    ve1->setDefaultProfilePreset(1280, 720, 25, dai::VideoEncoderProperties::Profile::H264_MAIN);
    ve2->setDefaultProfilePreset(3840, 2160, 25, dai::VideoEncoderProperties::Profile::H265_MAIN);
    ve3->setDefaultProfilePreset(1280, 720, 25, dai::VideoEncoderProperties::Profile::H264_MAIN);

    // Create outputs
    monoCam->out.link(ve1->input);
    colorCam->video.link(ve2->input);
    monoCam2->out.link(ve3->input);

    ve1->bitstream.link(ve1Out->input);
    ve2->bitstream.link(ve2Out->input);
    ve3->bitstream.link(ve3Out->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(p);
    // Start pipeline
    d.startPipeline();

    // Output queues will be used to get the encoded data from the output defined above
    auto outQ1 = d.getOutputQueue("ve1Out", 30, true);
    auto outQ2 = d.getOutputQueue("ve2Out", 30, true);
    auto outQ3 = d.getOutputQueue("ve3Out", 30, true);

    // The .h264 / .h265 files are raw stream files (not playable yet)
    auto videoFile1 = std::ofstream("mono1.h264", std::ios::binary);
    auto videoFile2 = std::ofstream("color.h265", std::ios::binary);
    auto videoFile3 = std::ofstream("mono2.h264", std::ios::binary);
    std::cout << "Press Ctrl+C to stop encoding..." << std::endl;

    while(alive) {
        auto out1 = outQ1->get<dai::ImgFrame>();
        videoFile1.write((char*)out1->getData().data(), out1->getData().size());
        auto out2 = outQ2->get<dai::ImgFrame>();
        videoFile2.write((char*)out2->getData().data(), out2->getData().size());
        auto out3 = outQ3->get<dai::ImgFrame>();
        videoFile3.write((char*)out3->getData().data(), out3->getData().size());
    }

    std::cout << "To view the encoded data, convert the stream file (.h264/.h265) into a video file (.mp4), using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate 25 -i mono1.h264 -c copy mono1.mp4" << std::endl;
    std::cout << "ffmpeg -framerate 25 -i mono2.h264 -c copy mono2.mp4" << std::endl;
    std::cout << "ffmpeg -framerate 25 -i color.h265 -c copy color.mp4" << std::endl;

    return 0;
}
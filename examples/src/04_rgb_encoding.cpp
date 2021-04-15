
#include <csignal>
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

    // Keyboard interrupt (Ctrl + C) detected
static bool alive = true;
static void sigintHandler(int signum) {
    alive = false;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    signal(SIGINT, &sigintHandler);

    std::string h265Path("video.h265");

    // If path specified, use that
    if(argc > 1) {
        h265Path = std::string(argv[1]);
    }

    dai::Pipeline p;

    // Define a source - color camera
    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xout = p.create<dai::node::XLinkOut>();
    auto videnc = p.create<dai::node::VideoEncoder>();

    xout->setStreamName("h265");
    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);

    videnc->setDefaultProfilePreset(3840, 2160, 30, dai::VideoEncoderProperties::Profile::H265_MAIN);

    // Create outputs
    colorCam->video.link(videnc->input);
    videnc->bitstream.link(xout->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(p);
    // Start pipeline
    d.startPipeline();

    // Output queue will be used to get the encoded data from the output defined above
    auto q = d.getOutputQueue("h265", 30, true);

    // The .h265 file is a raw stream file (not playable yet)
    auto videoFile = std::fstream(h265Path, std::ios::out | std::ios::binary);
    std::cout << "Press Ctrl+C to stop encoding..." << std::endl;

    while(alive) {
        auto h264Packet = q->get<dai::ImgFrame>();
        videoFile.write((char*)h264Packet->getData().data(), h264Packet->getData().size());
    }
    videoFile.close();

    std::cout << "To view the encoded data, convert the stream file " << h265Path << " into a video file (.mp4) using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate " << colorCam->getFps() << " -i " << h265Path << " -c copy video.mp4" << std::endl;

    return 0;
}

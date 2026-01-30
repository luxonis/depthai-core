#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <optional>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

constexpr std::pair<int, int> SIZE = {1280, 720};
constexpr int FPS = 480;

std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

class VideoSaver : public dai::node::CustomNode<VideoSaver> {
   public:
    VideoSaver() : fileHandle("video_hfr.encoded", std::ios::binary) {
        if(!fileHandle.is_open()) {
            throw std::runtime_error("Could not open video_hfr.encoded for writing");
        }
    }

    ~VideoSaver() {
        if(fileHandle.is_open()) {
            fileHandle.close();
        }
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> message) override {
        if(!fileHandle.is_open()) return nullptr;

        auto frame = message->get<dai::EncodedFrame>("data");
        unsigned char* frameData = frame->getData().data();
        size_t frameSize = frame->getData().size();
        fileHandle.write(reinterpret_cast<const char*>(frameData), frameSize);

        return nullptr;
    }

   private:
    std::ofstream fileHandle;
};

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform != dai::Platform::RVC4) {
        throw std::runtime_error("This example is only supported on RVC4 devices");
    }

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto output = camRgb->requestOutput(SIZE, std::nullopt, dai::ImgResizeMode::CROP, static_cast<float>(FPS));

    // ImageManip is added to workaround a limitation with VideoEncoder with native resolutions
    // This limitation will be lifted in the future
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->initialConfig->setOutputSize(SIZE.first, SIZE.second + 10);
    imageManip->setMaxOutputFrameSize(static_cast<int>(SIZE.first * (SIZE.second + 10) * 1.6));
    output->link(imageManip->inputImage);
    auto encodedInput = imageManip->out;

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);

    auto encoded = pipeline.create<dai::node::VideoEncoder>();
    encoded->setDefaultProfilePreset(static_cast<float>(FPS), dai::VideoEncoderProperties::Profile::H264_MAIN);
    encodedInput.link(encoded->input);
    encoded->out.link(benchmarkIn->input);

    auto saver = pipeline.create<VideoSaver>();
    encoded->out.link(saver->inputs["data"]);

    pipeline.start();

    std::cout << "Started to save video to video_hfr.encoded" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    while(pipeline.isRunning() && !quitEvent) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    pipeline.stop();
    pipeline.wait();

    std::cout << "To view the encoded data, convert the stream file (.encoded) into a video file (.mp4) using a command below:" << std::endl;
    std::cout << "ffmpeg -framerate " << FPS << " -i video_hfr.encoded -c copy video_hfr.mp4" << std::endl;

    std::cout << "If the FPS is not set correctly, you can ask ffmpeg to generate it with the command below" << std::endl;

    std::cout << "\nffmpeg -fflags +genpts -r " << FPS << " -i video_hfr.encoded \\\n  -vsync cfr -fps_mode cfr \\\n  -video_track_timescale " << FPS
              << "00 \\\n  -c:v copy \\\n  video_hfr.mp4\n";

    return 0;
}

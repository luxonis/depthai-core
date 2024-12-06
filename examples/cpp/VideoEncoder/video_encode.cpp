#include <csignal>
#include <iostream>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"

class VideoSaver : public dai::NodeCRTP<dai::node::HostNode, VideoSaver> {
   public:
    Input& input = inputs["in"];
    std::ofstream file;

    VideoSaver() {
        file.open("video.encoded", std::ios::out | std::ios ::binary);
    }

    std::shared_ptr<VideoSaver> build(Node::Output& out) {
        out.link(input);
        return std::static_pointer_cast<VideoSaver>(shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto frame = in->get<dai::EncodedFrame>("in");
        auto span = frame->getData();
        file.write((char*)span.data(), span.size());
        return nullptr;
    }
};

static std::atomic<bool> alive{true};
static std::condition_variable sigCv;
static std::mutex sigMtx;
static void sigintHandler(int signum) {
    alive = false;
    sigCv.notify_all();
}

int main(int argc, char** argv) {
    dai::Pipeline pipeline;
    auto camNode = pipeline.create<dai::node::Camera>()->build();
    auto* camOut = camNode->requestOutput({640, 480}, dai::ImgFrame::Type::NV12);
    if(camOut == nullptr) {
        std::cerr << "Error creating camera output, exiting with 1\n";
        return 1;
    }
    auto encNode = pipeline.create<dai::node::VideoEncoder>()->build(*camOut);

    dai::VideoEncoderProperties::Profile profile = dai::VideoEncoderProperties::Profile::MJPEG;
    int quality = 30;
    bool lossless = false;
    int bitrate = 0;

    encNode->setProfile(profile);
    encNode->setBitrate(bitrate);
    encNode->setQuality(quality);
    encNode->setLossless(lossless);
    encNode->setKeyframeFrequency(30);

    auto saver = pipeline.create<VideoSaver>()->build(encNode->out);

    std::signal(SIGINT, &sigintHandler);
    pipeline.start();
    std::cout << "Started to save video to video.encoded\n" << std::flush;
    std::cout << "Press Ctrl+C to stop\n" << std::flush;
    while(alive) {
        std::unique_lock<std::mutex> lk(sigMtx);
        sigCv.wait(lk, [] { return !alive; });
    }
    pipeline.stop();
    pipeline.wait();

    return 0;
}

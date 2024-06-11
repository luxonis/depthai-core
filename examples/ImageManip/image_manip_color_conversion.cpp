#include "depthai/depthai.hpp"
#include "opencv2/opencv.hpp"

constexpr int NUM_FRAMES_PER_CONFIG = 100;

using Type = dai::ImgFrame::Type;

int main(int argc, char** argv) {
    assert(argc >= 2);

    dai::Pipeline pipeline;

    auto xlinkIn = pipeline.create<dai::node::XLinkIn>();
    auto xlinkInConfig = pipeline.create<dai::node::XLinkIn>();
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkIn->setStreamName("in");
    xlinkIn->setStreamName("inConfig");
    xlinkOut->setStreamName("out");
    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888i);

    xlinkIn->out.link(imageManip->inputImage);
    xlinkInConfig->out.link(imageManip->inputConfig);
    imageManip->out.link(xlinkOut->input);

    auto device = pipeline.getDevice();
    auto in = device->getInputQueue("in");
    auto inConfig = device->getInputQueue("inConfig");
    auto out = device->getOutputQueue("out");

    pipeline.start();

    std::vector<Type> supportedTypes = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::NV12};
    for(const auto from : supportedTypes) {
        for (const auto to : supportedTypes) {
            for(unsigned int i = 0; i < NUM_FRAMES_PER_CONFIG * (argc - 1); i++) {
                cv::Mat frame = cv::imread(argv[i / NUM_FRAMES_PER_CONFIG + 1], cv::IMREAD_COLOR);
                if(frame.empty()) {
                    std::cerr << "File not found: " << argv[i / NUM_FRAMES_PER_CONFIG + 1] << std::endl;
                }
                dai::ImgFrame imgFrame;
                // TODO: cv set frame
            }
        }
    }

    pipeline.stop();
}

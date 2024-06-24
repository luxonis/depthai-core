#include <memory>
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "opencv2/opencv.hpp"

constexpr int NUM_FRAMES_PER_CONFIG = 2;

using Type = dai::ImgFrame::Type;

int main(int argc, char** argv) {
    std::shared_ptr<dai::Device> device = nullptr;
    if(argc <= 1) {
        device = std::make_shared<dai::Device>();
    } else {
        device = std::make_shared<dai::Device>(argv[1]);
    }
    dai::Pipeline pipeline(device);

    auto imageManip = pipeline.create<dai::node::ImageManipV2>();

    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888i);
    imageManip->setMaxOutputFrameSize(4803988);

    auto inImg = imageManip->inputImage.createInputQueue();
    auto outImg = imageManip->out.createOutputQueue();
    auto inConfig = imageManip->inputConfig.createInputQueue();

    pipeline.start();

    std::vector<Type> supportedTypes = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::NV12, Type::YUV420p};
    for(const auto from : supportedTypes) {
        for (const auto to : supportedTypes) {
            for(unsigned int i = 0; i < NUM_FRAMES_PER_CONFIG * (argc - 1); i++) {
                cv::Mat frame = cv::imread(argv[i / NUM_FRAMES_PER_CONFIG + 1], cv::IMREAD_COLOR);
                cv::resize(frame, frame, cv::Size(640, 480));
                if(frame.empty()) {
                    std::cerr << "File not found: " << argv[i / NUM_FRAMES_PER_CONFIG + 1] << std::endl;
                }
                dai::ImgFrame imgFrame;
                imgFrame.setCvFrame(frame, from);
                dai::ImageManipConfigV2 config;
                config.setFrameType(to);
                inConfig->send(std::make_shared<dai::ImageManipConfigV2>(config));
                inImg->send(std::make_shared<dai::ImgFrame>(imgFrame));

                auto out = outImg->get<dai::ImgFrame>();
                // Horizontally merge frame and out->getCvFrame()
                cv::Mat cvOut = out->getCvFrame();
                cv::Mat merged;
                cv::hconcat(frame, cvOut, merged);
                cv::imshow("Frame", merged);
                cv::waitKey(50);
            }
        }
    }

    pipeline.stop();
}

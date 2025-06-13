#include <memory>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "opencv2/opencv.hpp"

constexpr int NUM_FRAMES_PER_CONFIG = 1;

using Type = dai::ImgFrame::Type;

int main(int argc, char** argv) {
    std::vector<std::string> filenames;
    for(int i = 1; i < argc; i++) {
        filenames.push_back(argv[i]);
    }
    if(filenames.empty()) filenames.push_back("img.png");
    dai::Pipeline pipeline(false);

    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->setRunOnHost();

    imageManip->setMaxOutputFrameSize(4803988);

    auto inImg = imageManip->inputImage.createInputQueue();
    auto outImg = imageManip->out.createOutputQueue();
    auto inConfig = imageManip->inputConfig.createInputQueue();

    pipeline.start();

    std::vector<Type> supportedTypesFrom = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::NV12, Type::YUV420p};
    std::vector<Type> supportedTypesTo = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::NV12, Type::YUV420p, Type::GRAY8};
    for(const auto from : supportedTypesFrom) {
        for(const auto to : supportedTypesTo) {
            for(unsigned int i = 0; i < NUM_FRAMES_PER_CONFIG * (argc - 1); i++) {
                cv::Mat frame = cv::imread(filenames[i / NUM_FRAMES_PER_CONFIG], cv::IMREAD_COLOR);
                if(frame.empty()) {
                    std::cerr << "File not found: " << filenames[i / NUM_FRAMES_PER_CONFIG] << std::endl;
                }
                cv::resize(frame, frame, cv::Size(640, 480));
                dai::ImgFrame imgFrame;
                imgFrame.setCvFrame(frame, from);
                dai::ImageManipConfig config;
                config.setFrameType(to);

                inConfig->send(std::make_shared<dai::ImageManipConfig>(config));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                inImg->send(std::make_shared<dai::ImgFrame>(imgFrame));

                auto out = outImg->get<dai::ImgFrame>();
                assert(out != nullptr && out->getWidth() == frame.cols && out->getHeight() == frame.rows);

                // Horizontally merge frame and out->getCvFrame()
                cv::Mat cvOut = out->getCvFrame();
                // if cvOut is grayscale, convert to 3-channel
                if(cvOut.channels() == 1) {
                    cv::cvtColor(cvOut, cvOut, cv::COLOR_GRAY2BGR);
                }
                cv::Mat merged;
                cv::hconcat(frame, cvOut, merged);
                cv::imshow("Frame", merged);
                cv::waitKey(1000);
            }
        }
    }

    pipeline.stop();
}

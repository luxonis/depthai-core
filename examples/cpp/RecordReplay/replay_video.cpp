#include <argparse/argparse.hpp>
#include <filesystem>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>

#include "depthai/depthai.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    argparse::ArgumentParser program("replay_video", "1.0", argparse::default_arguments::help);
    program.add_argument("-v", "--inputVideo").default_value(std::string("test_video.mp4")).help("Input video name");
    program.add_argument("-m", "--inputMetadata").default_value(std::string("test_video.mcap")).help("Input metadata name");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    const std::filesystem::path inputVideo{program.get<std::string>("--inputVideo")};
    const std::filesystem::path inputMetadata{program.get<std::string>("--inputMetadata")};

    // Check if the input video file exists.
    if(!std::filesystem::exists(inputVideo)) {
        std::cout << "First record a video using the record_video example\n";
        std::cout << "Input video file not found: " << inputVideo.string() << "\n";
        return 1;
    }

    try {
        dai::Pipeline pipeline;
        auto replay = pipeline.create<dai::node::ReplayVideo>();
        replay->setReplayVideoFile(inputVideo);
        if(std::filesystem::exists(inputMetadata)) {
            replay->setReplayMetadataFile(inputMetadata);
        }
        replay->setOutFrameType(dai::ImgFrame::Type::NV12);
        replay->setLoop(false);

        auto videoOut = replay->out.createOutputQueue();

        pipeline.start();
        while(pipeline.isRunning() && replay->isRunning()) {
            std::shared_ptr<dai::ImgFrame> outFrame;
            try {
                outFrame = videoOut->get<dai::ImgFrame>();
            } catch(const dai::MessageQueue::QueueException&) {
                // Replay stopped the pipeline.
                break;
            }
            if(outFrame == nullptr) break;
            cv::imshow("video", outFrame->getCvFrame());
            if(cv::waitKey(1) == 'q') {
                std::cout << "Stopping pipeline\n";
                pipeline.stop();
                break;
            }
        }
        pipeline.stop();
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}

#include <csignal>
#include <filesystem>
#include <opencv2/highgui.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "utility.hpp"
#ifndef DEPTHAI_MERGED_TARGET
    #error This example needs OpenCV support, which is not available on your system
#endif

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main(int argc, char** argv) {
    // Register signal handler
    std::signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

    dai::RecordConfig config;
    config.outputDir = argc > 1 ? std::string(argv[1]) : getDefaultRecordingPath();
    // config.videoEncoding.enabled = true;  // Use video encoding
    // config.videoEncoding.bitrate = 0;     // Automatic
    // config.videoEncoding.profile = dai::VideoEncoderProperties::Profile::H264_MAIN;

    pipeline.enableHolisticRecord(config);

    auto camA = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* camAOut = camA->requestFullResolutionOutput();
    auto camB = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto* camBOut = camB->requestFullResolutionOutput();
    auto camC = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto* camCOut = camC->requestFullResolutionOutput();

    auto* viewFinderOut = camA->requestOutput({640, 480});

    auto imu = pipeline.create<dai::node::IMU>();

    auto display = pipeline.create<dai::node::Display>();

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 400);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    // Discard frames
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncAttempts(0);
    camAOut->link(sync->inputs["camA"]);
    camBOut->link(sync->inputs["camB"]);
    camCOut->link(sync->inputs["camC"]);
    imu->out.link(sync->inputs["imu"]);

    auto viewFinderQueue = viewFinderOut->createOutputQueue();

    pipeline.start();

    while(isRunning && pipeline.isRunning()) {
        auto frame = viewFinderQueue->get<dai::ImgFrame>();
        cv::imshow("Video", frame->getCvFrame());
        auto key = cv::waitKey(10);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();
}

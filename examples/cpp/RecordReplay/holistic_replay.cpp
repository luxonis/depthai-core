#include <argparse/argparse.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>

#include "depthai/depthai.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

// On RVC4 the pipeline builds one device ReplayVideo per camera and one
// ReplayMetadataOnly for the IMU, from a single uploaded recording.
// On RVC2 the recording is extracted and replayed on the host.
// Video and metadata counts must match. recording.tar from CI does not (119 vs 140).
int main(int argc, char** argv) {
    argparse::ArgumentParser program("holistic_replay", "1.0", argparse::default_arguments::help);
    program.add_argument("-s", "--source").default_value(std::string("recordings/recording.tar")).help("Recording path");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n" << program;
        return 1;
    }

    try {
        dai::Pipeline pipeline(true);
        auto device = pipeline.getDefaultDevice();
        pipeline.enableHolisticReplay(program.get<std::string>("--source"));

        // Define source and output.
        auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto* camRgbOut = camRgb->requestOutput({600, 400}, std::nullopt, dai::ImgResizeMode::CROP, 30.f);

        const auto imuName = device->getConnectedIMU();
        std::shared_ptr<dai::node::IMU> imu;
        std::shared_ptr<dai::MessageQueue> imuQueue;
        if(!imuName.empty()) {
            imu = pipeline.create<dai::node::IMU>();
            imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
            imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
            imu->setBatchReportThreshold(100);
            imuQueue = imu->out.createOutputQueue();
        }

        auto videoQueue = camRgbOut->createOutputQueue();

        // Connect to device and start pipeline.
        pipeline.start();
        while(pipeline.isRunning()) {
            auto videoIn = videoQueue->get<dai::ImgFrame>();
            if(videoIn == nullptr) continue;
            if(imuQueue) {
                auto imuData = imuQueue->get<dai::IMUData>();
                if(imuData) {
                    for(const auto& packet : imuData->packets) {
                        const auto& accel = packet.acceleroMeter;
                        const auto& gyro = packet.gyroscope;
                        std::cout << "IMU Accelerometer: " << accel.x << " " << accel.y << " " << accel.z << "\n";
                        std::cout << "IMU Gyroscope: " << gyro.x << " " << gyro.y << " " << gyro.z << "\n";
                    }
                }
            }

            // Get BGR frame from NV12 encoded video frame to show with opencv.
            // Visualizing the frame on slower hosts might have overhead.
            cv::imshow("video", videoIn->getCvFrame());

            if(cv::waitKey(1) == 'q') break;
        }
        pipeline.stop();
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}

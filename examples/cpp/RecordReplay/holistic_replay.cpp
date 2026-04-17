#include <opencv2/highgui.hpp>

#include "depthai/depthai.hpp"

#ifndef DEPTHAI_MERGED_TARGET
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    dai::Pipeline pipeline;

    pipeline.enableHolisticReplay(argc > 1 ? std::string(argv[1]) : "recording.tar");

    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* camOut = camRgb->requestOutput({600, 400});

    auto imu = pipeline.create<dai::node::IMU>();

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    auto videoQueue = camOut->createOutputQueue();
    auto q = imu->out.createOutputQueue();

    pipeline.start();

    auto start = std::chrono::steady_clock::now();

    while(pipeline.isRunning()) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        auto imuData = q->get<dai::IMUData>();

        cv::imshow("video", videoIn->getCvFrame());

        for(auto& imuPacket : imuData->packets) {
            auto& acceleroValues = imuPacket.acceleroMeter;
            auto& gyroValues = imuPacket.gyroscope;

            printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    pipeline.stop();
}

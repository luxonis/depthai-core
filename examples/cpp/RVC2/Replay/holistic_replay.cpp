#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/utility/RecordReplay.hpp"

#ifndef DEPTHAI_MERGED_TARGET
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    dai::Pipeline pipeline(true);
    auto cam = pipeline.create<dai::node::ColorCamera>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto display = pipeline.create<dai::node::Display>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    cam->setFps(30);

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

    cam->video.link(display->input);
    auto q = imu->out.createOutputQueue();

    pipeline.enableHolisticReplay(argc > 1 ? std::string(argv[1]) : "recording.tar.gz");

    pipeline.start();

    auto start = std::chrono::steady_clock::now();

    while(std::chrono::steady_clock::now() - start < std::chrono::seconds(15)) {
        auto imuData = q->get<dai::IMUData>();
        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto& acceleroValues = imuPacket.acceleroMeter;
            auto& gyroValues = imuPacket.gyroscope;

            printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    pipeline.stop();
}

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/utility/RecordReplay.hpp"

#ifndef DEPTHAI_MERGED_TARGET
    #error This example needs OpenCV support, which is not available on your system
#endif

int main(int argc, char** argv) {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto imu = pipeline.create<dai::node::IMU>();
    auto display = pipeline.create<dai::node::Display>();

    auto camOut = cam->requestOutput({720, 1280}, dai::ImgFrame::Type::NV12);

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(100);

    camOut->link(display->input);
    auto q = imu->out.createOutputQueue();

    dai::RecordConfig config;
    config.outputDir = argc > 1 ? std::string(argv[1]) : ".";
    config.videoEncoding.enabled = true; // Use video encoding
    config.videoEncoding.bitrate = 0; // Automatic
    config.videoEncoding.profile = dai::VideoEncoderProperties::Profile::H264_MAIN;

    pipeline.enableHolisticRecord(config);

    pipeline.start();

    auto start = std::chrono::steady_clock::now();

    try {
        while(std::chrono::steady_clock::now() - start < std::chrono::seconds(15)) {
            auto imuData = q->get<dai::IMUData>();
            auto imuPackets = imuData->packets;
            for(auto& imuPacket : imuPackets) {
                auto& acceleroValues = imuPacket.acceleroMeter;
                auto& gyroValues = imuPacket.gyroscope;

                // printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
                // printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch(...) {

    }

    pipeline.stop();
}

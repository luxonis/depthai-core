
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    // enable RAW_ACCELEROMETER and RAW_GYROSCOPE at 400 hz rate
    imu->enableIMUSensor({dai::IMUSensor::RAW_ACCELEROMETER, dai::IMUSensor::RAW_GYROSCOPE}, 400);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(pipeline);

    bool firstTs = false;

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();

    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto acceleroTs1 = imuPacket.rawAcceleroMeter.timestamp.get();
            auto gyroTs1 = imuPacket.rawGyroscope.timestamp.get();
            if(!firstTs) {
                baseTs = std::min(acceleroTs1, gyroTs1);
                firstTs = true;
            }
            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;

            printf("Accelerometer timestamp: %ld ms\n", duration_cast<milliseconds>(acceleroTs).count());
            printf(
                "Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", imuPacket.rawAcceleroMeter.x, imuPacket.rawAcceleroMeter.y, imuPacket.rawAcceleroMeter.z);
            printf("Gyroscope timestamp: %ld ms\n", duration_cast<milliseconds>(gyroTs).count());
            printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", imuPacket.rawGyroscope.x, imuPacket.rawGyroscope.y, imuPacket.rawGyroscope.z);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

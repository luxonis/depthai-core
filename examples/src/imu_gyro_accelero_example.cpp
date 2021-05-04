
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("imu");

    dai::IMUSensorConfig sensorConfig;
    sensorConfig.reportIntervalUs = 2500;  // 400hz
    sensorConfig.sensorId = dai::IMUSensorId::RAW_ACCELEROMETER;
    imu->enableIMUSensor(sensorConfig);
    sensorConfig.sensorId = dai::IMUSensorId::RAW_GYROSCOPE;
    imu->enableIMUSensor(sensorConfig);
    // above this threshold packets will be sent in batch of X, if the host is not blocked
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    imu->setMaxBatchReports(5);
    // WARNING, temporarily 6 is the max

    // Link plugins CAM -> XLINK
    imu->out.link(xlinkOut->input);

    return p;
}

int main() {
    using namespace std;
    using namespace std::chrono;

    auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    int firstTs = false;

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    int cnt = 0;
    while(1) {
        auto imuPacket = imuQueue->tryGet<dai::IMUData>();

        if(imuPacket) {
            auto imuDatas = imuPacket->imuDatas;
            for(auto& imuData : imuDatas) {
                auto acceleroTs1 = imuData.rawAcceleroMeter.timestamp.getTimestamp();
                auto gyroTs1 = imuData.rawGyroscope.timestamp.getTimestamp();
                if(!firstTs) {
                    baseTs = std::min(acceleroTs1, gyroTs1);
                    firstTs = true;
                }
                auto acceleroTs = acceleroTs1 - baseTs;
                auto gyroTs = gyroTs1 - baseTs;

                printf("Accelerometer timestamp: %ld ms\n", duration_cast<milliseconds>(acceleroTs).count());
                printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", imuData.rawAcceleroMeter.x, imuData.rawAcceleroMeter.y, imuData.rawAcceleroMeter.z);
                printf("Gyroscope timestamp: %ld ms\n", duration_cast<milliseconds>(gyroTs).count());
                printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", imuData.rawGyroscope.x, imuData.rawGyroscope.y, imuData.rawGyroscope.z);
            }
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

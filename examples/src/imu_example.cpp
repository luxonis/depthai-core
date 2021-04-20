
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

    // Link plugins CAM -> XLINK
    imu->out.link(xlinkOut->input);

    return p;
}

int main() {
    using namespace std;
    using namespace std::chrono;

    auto baseTs = steady_clock::now();
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    auto imuQueue = d.getOutputQueue("imu");

    while(1) {
        auto imuPacket = imuQueue->get<dai::IMUData>();

        auto imuDatas = imuPacket->imuDatas;
        for(auto& imuData : imuDatas) {
            auto dur = imuData.ts.getTimestamp() - baseTs;

            printf("Timestamp: %ld ms\n", duration_cast<milliseconds>(dur).count());

            printf("Accelero: %.3f %.3f %.3f \n", imuData.accelerometer.x, imuData.accelerometer.y, imuData.accelerometer.z);
            printf("Gyro: %.3f %.3f %.3f \n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        }
        int key = cv::waitKey(100);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

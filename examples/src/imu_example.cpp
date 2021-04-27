
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static bool testCamera = true;

dai::Pipeline createCameraPipeline() {
    dai::Pipeline p;

    auto imu = p.create<dai::node::IMU>();
    auto xlinkOut = p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("imu");

    dai::IMUSensorConfig sensorConfig;
    sensorConfig.reportIntervalUs = 2500;  // 400hz
    sensorConfig.sensorId = dai::IMUSensorId::IMU_ACCELEROMETER;
    imu->enableIMUSensor(sensorConfig);
    sensorConfig.sensorId = dai::IMUSensorId::IMU_GYROSCOPE_CALIBRATED;
    imu->enableIMUSensor(sensorConfig);
    // above this threshold packets will be sent in batch of 10, if the host is not blocked
    imu->setBatchReportThreshold(10);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    imu->setMaxBatchReports(50);

    // Link plugins CAM -> XLINK
    imu->out.link(xlinkOut->input);

    if(testCamera) {
        auto monoCam = p.create<dai::node::MonoCamera>();
        auto xlinkOut2 = p.create<dai::node::XLinkOut>();
        xlinkOut2->setStreamName("mono");

        // Set camera socket
        monoCam->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // Link plugins CAM -> XLINK
        monoCam->out.link(xlinkOut2->input);
    }
    return p;
}

int main() {
    using namespace std;
    using namespace std::chrono;

    auto baseTs = steady_clock::now();
    dai::Pipeline p = createCameraPipeline();
    dai::Device d(p);
    d.startPipeline();

    auto imuQueue = d.getOutputQueue("imu", 50, false);

    while(1) {
        if(testCamera) {
            auto monoQueue = d.getOutputQueue("mono", 4, false);
            auto imgFrame = monoQueue->tryGet<dai::ImgFrame>();

            if(imgFrame) {
                auto latencyMs = imgFrame->getTimestamp() - baseTs;
                printf("=== Frame - Timestamp %ld ms\n", duration_cast<milliseconds>(latencyMs).count());
            }
        }
        auto imuPacket = imuQueue->tryGet<dai::IMUData>();

        if(imuPacket) {
            auto imuDatas = imuPacket->imuDatas;
            for(auto& imuData : imuDatas) {
                auto dur = imuData.ts.getTimestamp() - baseTs;

                printf("Timestamp: %ld ms\n", duration_cast<milliseconds>(dur).count());

                printf("Accelero: %.3f %.3f %.3f \n", imuData.accelerometer.x, imuData.accelerometer.y, imuData.accelerometer.z);
                printf("Gyro: %.3f %.3f %.3f \n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
            }
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

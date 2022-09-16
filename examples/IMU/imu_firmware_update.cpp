
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    std::cout << "Warning! Flashing IMU firmware can potentially soft brick your device and should be done with caution." << std::endl;
    std::cout << "Do not unplug your device while the IMU firmware is flashing." << std::endl;
    std::cout << "Type 'y' and press enter to proceed, otherwise exits: ";
    std::cin.ignore();
    if(std::cin.get() != 'y') {
        std::cout << "Prompt declined, exiting..." << std::endl;
        return -1;
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    imu->enableFirmwareUpdate(true);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(pipeline);

    bool firstTs = false;

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();

    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto& acceleroValues = imuPacket.acceleroMeter;
            auto& gyroValues = imuPacket.gyroscope;

            auto acceleroTs1 = acceleroValues.timestamp.get();
            auto gyroTs1 = gyroValues.timestamp.get();
            if(!firstTs) {
                baseTs = std::min(acceleroTs1, gyroTs1);
                firstTs = true;
            }

            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;

            printf("Accelerometer timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(acceleroTs).count()));
            printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            printf("Gyroscope timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(gyroTs).count()));
            printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

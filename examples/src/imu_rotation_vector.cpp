
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

    // Properties
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR}, 400);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    dai::Device d(pipeline);

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = steady_clock::now();

    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto rvTs = imuPacket.rotationVector.timestamp.get() - baseTs;
            auto& rVvalues = imuPacket.rotationVector;
            printf("Rotation vector timestamp: %ld ms\n", duration_cast<milliseconds>(rvTs).count());

            printf(
                "Quaternion: i: %.3f j: %.3f k: %.3f real: %.3f\n"
                "Accuracy (rad): %.3f \n",
                rVvalues.i,
                rVvalues.j,
                rVvalues.k,
                rVvalues.real,
                rVvalues.accuracy);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}

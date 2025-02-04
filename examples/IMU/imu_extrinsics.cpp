#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


std::array<float, 3> transformIMUVector(
    const std::array<float, 3>& vURB,
    const std::vector<std::vector<float>>& extrinsics)
{
    // Safety checks
    if (extrinsics.size() != 4 || extrinsics[0].size() != 4) {
        throw std::runtime_error("extrinsics must be a 4x4 matrix!");
    }

    // Extract the top-left 3x3 rotation R (row-major)
    // R(i,j) = extrinsics[i][j] for i,j in [0..2].
    std::array<float, 3> vFLU;
    vFLU[0] = extrinsics[0][0] * vURB[0]
            + extrinsics[0][1] * vURB[1]
            + extrinsics[0][2] * vURB[2];
    vFLU[1] = extrinsics[1][0] * vURB[0]
            + extrinsics[1][1] * vURB[1]
            + extrinsics[1][2] * vURB[2];
    vFLU[2] = extrinsics[2][0] * vURB[0]
            + extrinsics[2][1] * vURB[1]
            + extrinsics[2][2] * vURB[2];

    return vFLU;
}

int main() {
    using namespace std;

    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(pipeline);

    auto imuExtr = d.readCalibration().getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_B, true);


    auto imuQueue = d.getOutputQueue("imu", 50, false);
    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto& acceleroValues = imuPacket.acceleroMeter;

            std::array<float, 3> vURB = {acceleroValues.x, acceleroValues.y, acceleroValues.z};
            std::array<float, 3> vRDF = transformIMUVector(vURB, imuExtr);
            std::cout << "###############" << std::endl;
            std::cout << "vURB = " << vURB[0] << ", " << vURB[1] << ", " << vURB[2] << " -> vRDF = {" << vRDF[0] << ", " << vRDF[1] << ", " << vRDF[2] << "}" << std::endl;
            std::cout << "###############" << std::endl;
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}

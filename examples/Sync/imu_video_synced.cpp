#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    dai::Device device;

    auto imuType = device.getConnectedIMU();
    auto imuFirmwareVersion = device.getIMUFirmwareVersion();
    std::cout << "IMU type: " << imuType << ", firmware version: " << imuFirmwareVersion << std::endl;

    if(imuType != "BNO086") {
        std::cout << "Rotation vector output is supported only by BNO086!" << std::endl;
        return 1;
    }

    dai::Pipeline pipeline;

    auto colorCamera = pipeline.create<dai::node::ColorCamera>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto xoutGroup = pipeline.create<dai::node::XLinkOut>();

    xoutGroup->setStreamName("xout");

    colorCamera->setCamera("color");

    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 120);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    sync->setSyncThreshold(std::chrono::milliseconds(10));
    sync->setSyncAttempts(-1);  // Infinite attempts

    colorCamera->video.link(sync->inputs["video"]);
    imu->out.link(sync->inputs["imu"]);

    sync->out.link(xoutGroup->input);

    device.startPipeline(pipeline);

    auto groupQueue = device.getOutputQueue("xout", 3, false);

    while(true) {
        auto groupMessage = groupQueue->get<dai::MessageGroup>();
        auto imuData = groupMessage->get<dai::IMUData>("imu");
        auto colorData = groupMessage->get<dai::ImgFrame>("video");
        auto timeDifference = imuData->getTimestampDevice() - colorData->getTimestampDevice();
        auto timeDifferenceUs = std::chrono::duration_cast<std::chrono::microseconds>(timeDifference).count();

        std::cout << "Time difference between messages is: " << std::abs(timeDifferenceUs / 1000.0) << " ms" << std::endl;

        for(auto& packet : imuData->packets) {
            auto& rv = packet.rotationVector;

            printf(
                "Quaternion: i: %.3f j: %.3f k: %.3f real: %.3f\n"
                "Accuracy (rad): %.3f \n",
                rv.i,
                rv.j,
                rv.k,
                rv.real,
                rv.rotationVectorAccuracy);
        }

        cv::imshow("Color", colorData->getCvFrame());
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}

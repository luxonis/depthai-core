#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

constexpr double windowSeconds = 10.0;
constexpr double printUpdateHz = 4.0;
constexpr double kPi = 3.14159265358979323846;

std::atomic<bool> quitEvent(false);

const std::vector<std::vector<float>> kYZSwapRotation = {
    {0.0f, 1.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

struct Sample {
    double seconds;
    double x;
    double y;
    double z;
    float i;
    float j;
    float k;
    float real;
    float accuracy;
};

std::tuple<double, double, double> quaternionToEulerXYZ(float i, float j, float k, float real) {
    const double sinrCosp = 2.0 * (real * i + j * k);
    const double cosrCosp = 1.0 - 2.0 * (i * i + j * j);
    const double x = std::atan2(sinrCosp, cosrCosp) * 180.0 / kPi;

    double sinp = 2.0 * (real * j - k * i);
    sinp = std::clamp(sinp, -1.0, 1.0);
    const double y = std::asin(sinp) * 180.0 / kPi;

    const double sinyCosp = 2.0 * (real * k + i * j);
    const double cosyCosp = 1.0 - 2.0 * (j * j + k * k);
    const double z = std::atan2(sinyCosp, cosyCosp) * 180.0 / kPi;

    return {x, y, z};
}

void signalHandler(int) {
    quitEvent = true;
}

dai::CameraBoardSocket resolveImuExtrinsicsDestination(const dai::EepromData& eepromData) {
    const auto socket = eepromData.imuExtrinsics.toCameraSocket;
    if(socket != dai::CameraBoardSocket::AUTO) {
        return socket;
    }

    if(eepromData.cameraData.find(dai::CameraBoardSocket::CAM_A) != eepromData.cameraData.end()) {
        return dai::CameraBoardSocket::CAM_A;
    }
    if(!eepromData.cameraData.empty()) {
        return eepromData.cameraData.begin()->first;
    }

    return dai::CameraBoardSocket::CAM_A;
}

bool rotationVectorSupported(const std::shared_ptr<dai::Device>& device) {
    const auto imuName = device->getConnectedIMU();
    return imuName.rfind("BMI", 0) != 0;
}

std::array<double, 3> windowSpans(const std::deque<Sample>& samples) {
    const auto [minX, maxX] = std::minmax_element(samples.begin(), samples.end(), [](const auto& lhs, const auto& rhs) { return lhs.x < rhs.x; });
    const auto [minY, maxY] = std::minmax_element(samples.begin(), samples.end(), [](const auto& lhs, const auto& rhs) { return lhs.y < rhs.y; });
    const auto [minZ, maxZ] = std::minmax_element(samples.begin(), samples.end(), [](const auto& lhs, const auto& rhs) { return lhs.z < rhs.z; });

    return {maxX->x - minX->x, maxY->y - minY->y, maxZ->z - minZ->z};
}

}  // namespace

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    const auto imuName = device->getConnectedIMU();
    std::cout << "IMU type: " << imuName << ", firmware version: " << device->getIMUFirmwareVersion().toString() << std::endl;
    if(!rotationVectorSupported(device)) {
        std::cout << "Device HW does not support ROTATION_VECTOR. Connected IMU: " << imuName << " (BMI IMU detected)." << std::endl;
        return 0;
    }

    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    auto calibration = device->readCalibration();
    const auto eepromData = calibration.getEepromData();
    const auto& imuExtrinsics = eepromData.imuExtrinsics;
    const auto destinationSocket = resolveImuExtrinsicsDestination(eepromData);
    calibration.setImuExtrinsics(destinationSocket,
                                 kYZSwapRotation,
                                 {imuExtrinsics.translation.x, imuExtrinsics.translation.y, imuExtrinsics.translation.z},
                                 {imuExtrinsics.specTranslation.x, imuExtrinsics.specTranslation.y, imuExtrinsics.specTranslation.z});
    device->setCalibration(calibration);

    pipeline.start();
    std::cout << "Applied runtime IMU extrinsics Y/Z swap relative to " << destinationSocket << "." << std::endl;
    std::cout << "Rotation matrix: [[0, 1, 0], [1, 0, 0], [0, 0, 1]]" << std::endl;
    std::cout << "Rotation vector stream started on IMU " << imuName << "." << std::endl;
    std::cout << "Move the device around each axis and watch the rolling XYZ angle spans respond." << std::endl;

    std::cout << std::fixed << std::setprecision(6);
    const auto startTime = std::chrono::steady_clock::now();
    auto lastPrintTime = startTime;
    std::deque<Sample> samples;

    while(pipeline.isRunning() && !quitEvent) {
        auto imuData = imuQueue->tryGet<dai::IMUData>();
        if(imuData == nullptr) continue;

        for(const auto& imuPacket : imuData->packets) {
            const auto rotationVector = imuPacket.rotationVector;
            const auto [x, y, z] = quaternionToEulerXYZ(rotationVector.i, rotationVector.j, rotationVector.k, rotationVector.real);
            const auto now = std::chrono::steady_clock::now();
            const double sampleSeconds = std::chrono::duration<double>(now - startTime).count();

            samples.push_back(
                {sampleSeconds, x, y, z, rotationVector.i, rotationVector.j, rotationVector.k, rotationVector.real, rotationVector.rotationVectorAccuracy});

            while(!samples.empty() && sampleSeconds - samples.front().seconds > windowSeconds) {
                samples.pop_front();
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if(!samples.empty() && std::chrono::duration<double>(now - lastPrintTime).count() >= 1.0 / printUpdateHz) {
            const auto& latest = samples.back();
            const auto spans = windowSpans(samples);
            std::cout << "Window " << std::setw(4) << windowSeconds << "s span [deg]: x=" << std::setw(8) << spans[0] << " y=" << std::setw(8) << spans[1]
                      << " z=" << std::setw(8) << spans[2] << " | latest xyz=[" << std::setw(8) << latest.x << ", " << std::setw(8) << latest.y << ", "
                      << std::setw(8) << latest.z << "] quat=[" << std::setw(9) << latest.i << ", " << std::setw(9) << latest.j << ", " << std::setw(9)
                      << latest.k << ", " << std::setw(9) << latest.real << "] acc=" << latest.accuracy << " rad" << std::endl;
            lastPrintTime = now;
        }
    }

    pipeline.stop();
    pipeline.wait();
    return 0;
}

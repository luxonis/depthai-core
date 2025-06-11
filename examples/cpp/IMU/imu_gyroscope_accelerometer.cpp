#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Helper function to convert time delta to milliseconds
float timeDeltaToMilliS(const std::chrono::steady_clock::duration& delta) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(delta).count();
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();

    // Enable ACCELEROMETER_RAW at 480 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 480);
    // Enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

    // Set batch report threshold and max batch reports
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Create output queue
    auto imuQueue = imu->out.createOutputQueue(50, false);

    // Start pipeline
    pipeline.start();
    std::cout << "IMU pipeline started. Press Ctrl+C to stop." << std::endl;

    // Set up output formatting
    std::cout << std::fixed << std::setprecision(6);

    while(pipeline.isRunning() && !quitEvent) {
        auto imuData = imuQueue->get<dai::IMUData>();
        if(imuData == nullptr) continue;

        for(const auto& imuPacket : imuData->packets) {
            auto acceleroValues = imuPacket.acceleroMeter;
            auto gyroValues = imuPacket.gyroscope;

            auto acceleroTs = acceleroValues.getTimestamp();
            auto gyroTs = gyroValues.getTimestamp();

            // Print accelerometer data
            std::cout << "Accelerometer timestamp: " << acceleroTs.time_since_epoch().count() << std::endl;
            std::cout << "Latency [ms]: " << timeDeltaToMilliS(std::chrono::steady_clock::now() - acceleroValues.getTimestamp()) << std::endl;
            std::cout << "Accelerometer [m/s^2]: x: " << acceleroValues.x << " y: " << acceleroValues.y << " z: " << acceleroValues.z << std::endl;

            // Print gyroscope data
            std::cout << "Gyroscope timestamp: " << gyroTs.time_since_epoch().count() << std::endl;
            std::cout << "Gyroscope [rad/s]: x: " << gyroValues.x << " y: " << gyroValues.y << " z: " << gyroValues.z << std::endl;
        }
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    return 0;
}
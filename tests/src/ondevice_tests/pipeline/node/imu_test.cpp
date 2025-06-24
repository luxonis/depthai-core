#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <initializer_list>

#include "depthai/depthai.hpp"
#include "depthai/properties/IMUProperties.hpp"

void basicIMUTest(float fps, std::initializer_list<dai::IMUSensor> sensors, float maxEpsilon = 0.3f) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    for(auto sensor : sensors) {
        imu->enableIMUSensor(sensor, fps);
    }
    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    imu->out.link(benchmarkIn->input);
    auto reportQueue = benchmarkIn->report.createOutputQueue();
    p.start();
    for(int i = 0; i < 10; i++) {
        auto reportData = reportQueue->get<dai::BenchmarkReport>();
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
        REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(maxEpsilon));
        REQUIRE(reportData->averageLatency > 0.0);
        REQUIRE(reportData->averageLatency < 1.0);  // Sanity check that the latency measurement works correctly
    }
}

TEST_CASE("Test IMU, 30Hz, accelerometer, gyroscope") {
    basicIMUTest(30.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW});
}

TEST_CASE("Test IMU, 100Hz, accelerometer, gyroscope") {
    basicIMUTest(100.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW});
}

TEST_CASE("Test IMU, all sensors") {
    basicIMUTest(50.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW, dai::IMUSensor::MAGNETOMETER_RAW, dai::IMUSensor::ROTATION_VECTOR});
}

TEST_CASE("Test IMU, gyroscope 480 Hz") {
    basicIMUTest(480.0f, {dai::IMUSensor::GYROSCOPE_RAW}, 0.8f);  // TODO(Morato) - debug why some devices need so much tolerance
}

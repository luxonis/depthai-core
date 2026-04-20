#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/properties/IMUProperties.hpp"

void basicIMUTest(float fps, std::initializer_list<dai::IMUSensor> sensors, float maxEpsilon = 0.3f) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    auto device = p.getDefaultDevice();
    const auto imuName = device->getConnectedIMU();

    std::vector<dai::IMUSensor> enabledSensors;
    enabledSensors.reserve(sensors.size());
    for(auto sensor : sensors) {
        if(imuName == "BMI270"
           && (sensor == dai::IMUSensor::MAGNETOMETER_RAW || sensor == dai::IMUSensor::MAGNETOMETER_CALIBRATED
               || sensor == dai::IMUSensor::MAGNETOMETER_UNCALIBRATED || sensor == dai::IMUSensor::ROTATION_VECTOR
               || sensor == dai::IMUSensor::GAME_ROTATION_VECTOR || sensor == dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR
               || sensor == dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR || sensor == dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR)) {
            continue;
        }
        enabledSensors.push_back(sensor);
    }

    if(enabledSensors.empty()) {
        SKIP("Connected IMU does not support any requested sensors for this test case");
    }

    for(auto sensor : enabledSensors) {
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
        if(!p.isHolisticReplayEnabled()) {
            REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(maxEpsilon));
            REQUIRE(reportData->averageLatency > 0.0);
            REQUIRE(reportData->averageLatency < 1.0);  // Sanity check that the latency measurement works correctly
        }
    }
}

TEST_CASE("Test IMU, 30Hz, accelerometer, gyroscope") {
    basicIMUTest(30.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW});
}

TEST_CASE("Test IMU, 100Hz, accelerometer, gyroscope") {
    basicIMUTest(100.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW});
}

TEST_CASE("Test IMU, 100Hz, accelerometer and gyroscope in the unified IMU frame") {
    basicIMUTest(100.0f, {dai::IMUSensor::ACCELEROMETER_UNCALIBRATED, dai::IMUSensor::GYROSCOPE_UNCALIBRATED});
}

TEST_CASE("Test IMU, all sensors") {
    basicIMUTest(50.0f, {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW, dai::IMUSensor::MAGNETOMETER_RAW, dai::IMUSensor::ROTATION_VECTOR});
}

TEST_CASE("Test IMU, gyroscope 480 Hz") {
    basicIMUTest(480.0f, {dai::IMUSensor::GYROSCOPE_RAW}, 0.8f);  // TODO(Morato) - debug why some devices need so much tolerance
}

namespace {

using Vec3 = std::array<float, 3>;

std::vector<std::vector<float>> makeAxisRotationCalibration(const std::array<float, 12>& matrix) {
    return {
        {matrix[0], matrix[1], matrix[2], matrix[3]},
        {matrix[4], matrix[5], matrix[6], matrix[7]},
        {matrix[8], matrix[9], matrix[10], matrix[11]},
    };
}

const std::vector<std::vector<float>> kIdentityCalibration = makeAxisRotationCalibration({
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
    0.0f,
});

const std::vector<std::vector<float>> kIdentityRotation = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

const std::vector<std::vector<float>> kTestNonIdentityMatrix = {
    {0.25f, -0.5f, 0.75f},
    {1.0f, 0.125f, -0.25f},
    {-0.375f, 0.625f, 0.875f},
};

Vec3 applyCalibration(const std::vector<std::vector<float>>& calibration, const Vec3& input) {
    return {
        calibration[0][0] * input[0] + calibration[0][1] * input[1] + calibration[0][2] * input[2] + calibration[0][3],
        calibration[1][0] * input[0] + calibration[1][1] * input[1] + calibration[1][2] * input[2] + calibration[1][3],
        calibration[2][0] * input[0] + calibration[2][1] * input[1] + calibration[2][2] * input[2] + calibration[2][3],
    };
}

Vec3 applyRotation(const std::vector<std::vector<float>>& rotation, const Vec3& input) {
    return {
        rotation[0][0] * input[0] + rotation[0][1] * input[1] + rotation[0][2] * input[2],
        rotation[1][0] * input[0] + rotation[1][1] * input[1] + rotation[1][2] * input[2],
        rotation[2][0] * input[0] + rotation[2][1] * input[1] + rotation[2][2] * input[2],
    };
}

Vec3 captureAverageAccelerometer(dai::IMUSensor accelerometerSensor = dai::IMUSensor::ACCELEROMETER_UNCALIBRATED,
                                 std::optional<std::vector<std::vector<float>>> calibration = std::nullopt,
                                 int warmupPacketCount = 20,
                                 int samplePacketCount = 1000) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(accelerometerSensor, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    auto device = p.getDefaultDevice();
    auto calibHandler = device->readCalibration();
    const auto& requestedCalibration = calibration.value_or(kIdentityCalibration);
    calibHandler.setAccelerometerCalibration(requestedCalibration);
    calibHandler.setGyroscopeCalibration(requestedCalibration);
    device->setCalibration(calibHandler);
    p.setCalibrationData(calibHandler);

    p.start();

    Vec3 sum = {0.0f, 0.0f, 0.0f};
    int collected = 0;
    int seen = 0;

    while(collected < samplePacketCount) {
        auto imuData = imuQueue->get<dai::IMUData>();
        REQUIRE(imuData != nullptr);

        for(const auto& imuPacket : imuData->packets) {
            if(seen++ < warmupPacketCount) {
                continue;
            }

            sum[0] += imuPacket.acceleroMeter.x;
            sum[1] += imuPacket.acceleroMeter.y;
            sum[2] += imuPacket.acceleroMeter.z;
            ++collected;

            if(collected >= samplePacketCount) {
                break;
            }
        }
    }

    p.stop();

    return {sum[0] / samplePacketCount, sum[1] / samplePacketCount, sum[2] / samplePacketCount};
}

Vec3 captureAverageGyroscope(std::optional<std::vector<std::vector<float>>> calibration = std::nullopt,
                             int warmupPacketCount = 20,
                             int samplePacketCount = 200) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_CALIBRATED, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    auto device = p.getDefaultDevice();
    auto calibHandler = device->readCalibration();
    const auto& requestedCalibration = calibration.value_or(kIdentityCalibration);
    calibHandler.setAccelerometerCalibration(requestedCalibration);
    calibHandler.setGyroscopeCalibration(requestedCalibration);
    device->setCalibration(calibHandler);
    p.setCalibrationData(calibHandler);

    p.start();

    Vec3 sum = {0.0f, 0.0f, 0.0f};
    int collected = 0;
    int seen = 0;

    while(collected < samplePacketCount) {
        auto imuData = imuQueue->get<dai::IMUData>();
        REQUIRE(imuData != nullptr);

        for(const auto& imuPacket : imuData->packets) {
            if(seen++ < warmupPacketCount) {
                continue;
            }

            sum[0] += imuPacket.gyroscope.x;
            sum[1] += imuPacket.gyroscope.y;
            sum[2] += imuPacket.gyroscope.z;
            ++collected;

            if(collected >= samplePacketCount) {
                break;
            }
        }
    }

    p.stop();

    return {sum[0] / samplePacketCount, sum[1] / samplePacketCount, sum[2] / samplePacketCount};
}

void requireVecNear(const Vec3& actual, const Vec3& expected, float margin = 5e-2f) {
    REQUIRE(actual[0] == Catch::Approx(expected[0]).margin(margin));
    REQUIRE(actual[1] == Catch::Approx(expected[1]).margin(margin));
    REQUIRE(actual[2] == Catch::Approx(expected[2]).margin(margin));
}

bool isIdentity3x3(const std::vector<std::vector<float>>& matrix, float tolerance = 1e-6f) {
    if(matrix.size() != 3) return false;
    for(int i = 0; i < 3; ++i) {
        if(matrix[i].size() != 3) return false;
    }

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            const float expected = (i == j) ? 1.0f : 0.0f;
            if(std::abs(matrix[i][j] - expected) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

void requireMatrixNear(const std::vector<std::vector<float>>& actual, const std::vector<std::vector<float>>& expected, float tolerance = 1e-6f) {
    REQUIRE(actual.size() == expected.size());
    for(size_t i = 0; i < actual.size(); ++i) {
        REQUIRE(actual[i].size() == expected[i].size());
        for(size_t j = 0; j < actual[i].size(); ++j) {
            REQUIRE(actual[i][j] == Catch::Approx(expected[i][j]).margin(tolerance));
        }
    }
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

void setImuExtrinsicsRotation(dai::CalibrationHandler& handler, const std::vector<std::vector<float>>& rotation) {
    const auto eepromData = handler.getEepromData();
    const auto& extrinsics = eepromData.imuExtrinsics;
    handler.setImuExtrinsics(resolveImuExtrinsicsDestination(eepromData),
                             rotation,
                             {extrinsics.translation.x, extrinsics.translation.y, extrinsics.translation.z},
                             {extrinsics.specTranslation.x, extrinsics.specTranslation.y, extrinsics.specTranslation.z});
}

void requireSameImuExtrinsics(const dai::CalibrationHandler& lhs, const dai::CalibrationHandler& rhs, float tolerance = 1e-6f) {
    const auto& lhsExtrinsics = lhs.getEepromData().imuExtrinsics;
    const auto& rhsExtrinsics = rhs.getEepromData().imuExtrinsics;

    REQUIRE(lhsExtrinsics.toCameraSocket == rhsExtrinsics.toCameraSocket);
    requireMatrixNear(lhsExtrinsics.rotationMatrix, rhsExtrinsics.rotationMatrix, tolerance);
    REQUIRE(lhsExtrinsics.translation.x == Catch::Approx(rhsExtrinsics.translation.x).margin(tolerance));
    REQUIRE(lhsExtrinsics.translation.y == Catch::Approx(rhsExtrinsics.translation.y).margin(tolerance));
    REQUIRE(lhsExtrinsics.translation.z == Catch::Approx(rhsExtrinsics.translation.z).margin(tolerance));
    REQUIRE(lhsExtrinsics.specTranslation.x == Catch::Approx(rhsExtrinsics.specTranslation.x).margin(tolerance));
    REQUIRE(lhsExtrinsics.specTranslation.y == Catch::Approx(rhsExtrinsics.specTranslation.y).margin(tolerance));
    REQUIRE(lhsExtrinsics.specTranslation.z == Catch::Approx(rhsExtrinsics.specTranslation.z).margin(tolerance));
}

Vec3 captureAverageAccelWithImuRotation(const std::vector<std::vector<float>>& rotation,
                                        dai::IMUSensor accelerometerSensor = dai::IMUSensor::ACCELEROMETER_CALIBRATED,
                                        int warmupPacketCount = 20,
                                        int samplePacketCount = 200) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(accelerometerSensor, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    auto device = p.getDefaultDevice();
    auto calibHandler = device->readCalibration();
    calibHandler.setAccelerometerCalibration(kIdentityCalibration);
    calibHandler.setGyroscopeCalibration(kIdentityCalibration);
    setImuExtrinsicsRotation(calibHandler, rotation);
    device->setCalibration(calibHandler);
    p.start();

    Vec3 sum = {0.0f, 0.0f, 0.0f};
    int collected = 0;
    int seen = 0;
    while(collected < samplePacketCount) {
        auto imuData = imuQueue->get<dai::IMUData>();
        REQUIRE(imuData != nullptr);
        for(const auto& imuPacket : imuData->packets) {
            if(seen++ < warmupPacketCount) continue;
            sum[0] += imuPacket.acceleroMeter.x;
            sum[1] += imuPacket.acceleroMeter.y;
            sum[2] += imuPacket.acceleroMeter.z;
            ++collected;
            if(collected >= samplePacketCount) break;
        }
    }
    p.stop();
    return {sum[0] / samplePacketCount, sum[1] / samplePacketCount, sum[2] / samplePacketCount};
}

struct ImuStreamRunResult {
    int packetCount = 0;
};

ImuStreamRunResult runImuWithCalibration(const dai::CalibrationHandler& calibration, int samplePacketCount = 20, int timeoutMs = 5000) {
    dai::Pipeline p;
    auto imu = p.create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    auto device = p.getDefaultDevice();
    REQUIRE(device != nullptr);

    device->setCalibration(calibration);
    p.start();

    int packetCount = 0;
    while(packetCount < samplePacketCount) {
        bool hasTimedOut = false;
        auto imuData = imuQueue->get<dai::IMUData>(std::chrono::milliseconds(timeoutMs), hasTimedOut);
        REQUIRE_FALSE(hasTimedOut);
        REQUIRE(imuData != nullptr);
        packetCount += static_cast<int>(imuData->packets.size());
    }

    p.stop();
    return {packetCount};
}

}  // namespace

TEST_CASE("Test IMU runtime calibration rotates accelerometer axes") {
    const auto baseline = captureAverageAccelerometer();

    SECTION("swap x and y with 90 degree rotation around z") {
        const auto rotation = makeAxisRotationCalibration({
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            -1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
        });

        const auto rotated = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, rotation);
        requireVecNear(rotated, applyCalibration(rotation, baseline));
    }

    SECTION("swap x and z with 90 degree rotation around y") {
        const auto rotation = makeAxisRotationCalibration({
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            -1.0f,
            0.0f,
            0.0f,
            0.0f,
        });

        const auto rotated = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, rotation);
        requireVecNear(rotated, applyCalibration(rotation, baseline));
    }

    SECTION("swap y and z with 90 degree rotation around x") {
        const auto rotation = makeAxisRotationCalibration({
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            -1.0f,
            0.0f,
            0.0f,
        });

        const auto rotated = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, rotation);
        requireVecNear(rotated, applyCalibration(rotation, baseline));
    }
}

TEST_CASE("Test IMU uncalibrated accelerometer ignores runtime affine calibration") {
    const auto baseline = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_UNCALIBRATED);
    const auto rotation = makeAxisRotationCalibration({
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        -1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
        0.125f,
    });
    const auto uncalibrated = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_UNCALIBRATED, rotation);
    requireVecNear(uncalibrated, baseline, 1e-2f);
}

TEST_CASE("Test IMU runtime calibration applies bias offsets") {
    const auto baselineAccel = captureAverageAccelerometer();
    const auto baselineGyro = captureAverageGyroscope();

    SECTION("bias on x axis") {
        const auto calibration = makeAxisRotationCalibration({
            1.0f,
            0.0f,
            0.0f,
            0.125f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
        });

        const auto accelBiased = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, calibration);
        const auto gyroBiased = captureAverageGyroscope(calibration);
        requireVecNear(accelBiased, applyCalibration(calibration, baselineAccel), 2e-2f);
        requireVecNear(gyroBiased, applyCalibration(calibration, baselineGyro), 2e-2f);
    }

    SECTION("bias on y axis") {
        const auto calibration = makeAxisRotationCalibration({
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            -0.125f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
        });

        const auto accelBiased = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, calibration);
        const auto gyroBiased = captureAverageGyroscope(calibration);
        requireVecNear(accelBiased, applyCalibration(calibration, baselineAccel), 2e-2f);
        requireVecNear(gyroBiased, applyCalibration(calibration, baselineGyro), 2e-2f);
    }

    SECTION("bias on z axis") {
        const auto calibration = makeAxisRotationCalibration({
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.125f,
        });

        const auto accelBiased = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_CALIBRATED, calibration);
        const auto gyroBiased = captureAverageGyroscope(calibration);
        requireVecNear(accelBiased, applyCalibration(calibration, baselineAccel), 2e-2f);
        requireVecNear(gyroBiased, applyCalibration(calibration, baselineGyro), 2e-2f);
    }
}

TEST_CASE("Test IMU getCalibration preserves non-identity IMU extrinsics") {
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    REQUIRE(device != nullptr);

    auto originalCalibration = device->getCalibration();
    auto randomCalibration = originalCalibration;
    setImuExtrinsicsRotation(randomCalibration, kTestNonIdentityMatrix);
    device->setCalibration(randomCalibration);

    const auto currentCalibration = device->getCalibration();
    requireMatrixNear(currentCalibration.getEepromData().imuExtrinsics.rotationMatrix, kTestNonIdentityMatrix);

    device->setCalibration(originalCalibration);
}

TEST_CASE("Test IMU getCalibration applies BNO identity fallback") {
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    REQUIRE(device != nullptr);

    const auto imuName = device->getConnectedIMU();
    if(imuName.rfind("BNO08", 0) != 0) {
        SKIP("Connected IMU is not a BNO08x variant");
    }

    auto originalCalibration = device->getCalibration();
    auto identityCalibration = originalCalibration;
    setImuExtrinsicsRotation(identityCalibration, kIdentityRotation);
    device->setCalibration(identityCalibration);

    const auto currentCalibration = device->getCalibration();
    REQUIRE_FALSE(isIdentity3x3(currentCalibration.getEepromData().imuExtrinsics.rotationMatrix));

    device->setCalibration(originalCalibration);
}

TEST_CASE("Test IMU readCalibration remains separate from runtime non-identity extrinsics") {
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    REQUIRE(device != nullptr);

    const auto originalReadCalibration = device->readCalibration();
    auto runtimeCalibration = device->getCalibration();
    setImuExtrinsicsRotation(runtimeCalibration, kTestNonIdentityMatrix);
    REQUIRE_NOTHROW(device->setCalibration(runtimeCalibration));

    const auto getBack = device->getCalibration();
    const auto readBack = device->readCalibration();

    requireMatrixNear(getBack.getEepromData().imuExtrinsics.rotationMatrix, kTestNonIdentityMatrix);
    requireSameImuExtrinsics(readBack, originalReadCalibration);

    device->setCalibration(originalReadCalibration);
}

TEST_CASE("Test IMU readCalibration can be loaded into runtime calibration") {
    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    REQUIRE(device != nullptr);

    const auto originalCalibration = device->getCalibration();
    const auto readBack = device->readCalibration();
    REQUIRE_NOTHROW(device->setCalibration(readBack));
    const auto getBack = device->getCalibration();
    const auto& effectiveRotation = readBack.getEepromData().imuExtrinsics.rotationMatrix;

    requireSameImuExtrinsics(readBack, getBack);

    const auto imuName = device->getConnectedIMU();
    if(imuName.rfind("BNO08", 0) == 0) {
        REQUIRE_FALSE(isIdentity3x3(effectiveRotation));
    } else {
        REQUIRE(effectiveRotation.size() == 3);
        for(const auto& row : effectiveRotation) {
            REQUIRE(row.size() == 3);
        }
    }

    device->setCalibration(originalCalibration);
}

TEST_CASE("Test setImuRotation applies extrinsics rotation to accelerometer via rotateImuVector") {
    const auto baselineRaw = captureAverageAccelWithImuRotation(kIdentityRotation, dai::IMUSensor::ACCELEROMETER_RAW);

    SECTION("90 degree rotation around Z swaps X and Y axes") {
        // R_z(90°): x' = -y, y' = x, z' = z
        const std::vector<std::vector<float>> rotZ90 = {
            {0.0f, -1.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };
        const auto rotated = captureAverageAccelWithImuRotation(rotZ90);
        requireVecNear(rotated, applyRotation(rotZ90, baselineRaw));
    }

    SECTION("90 degree rotation around Y swaps X and Z axes") {
        // R_y(90°): x' = z, y' = y, z' = -x
        const std::vector<std::vector<float>> rotY90 = {
            {0.0f, 0.0f, 1.0f},
            {0.0f, 1.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
        };
        const auto rotated = captureAverageAccelWithImuRotation(rotY90);
        requireVecNear(rotated, applyRotation(rotY90, baselineRaw));
    }

    SECTION("90 degree rotation around X swaps Y and Z axes") {
        // R_x(90°): x' = x, y' = z, z' = -y
        const std::vector<std::vector<float>> rotX90 = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            {0.0f, -1.0f, 0.0f},
        };
        const auto rotated = captureAverageAccelWithImuRotation(rotX90);
        requireVecNear(rotated, applyRotation(rotX90, baselineRaw));
    }
}

TEST_CASE("Test setImuRotation does not affect affine calibration path") {
    // ACCELEROMETER_UNCALIBRATED goes through rotateImuVector (extrinsics rotation) but
    // ignores imuCalibrationParams (affine calibration). Verify that setting only the
    // affine calibration matrix does not alter ACCELEROMETER_UNCALIBRATED readings.
    const auto baselineUncalib = captureAverageAccelWithImuRotation(kIdentityRotation, dai::IMUSensor::ACCELEROMETER_UNCALIBRATED);

    const auto withAffineOnly = captureAverageAccelerometer(dai::IMUSensor::ACCELEROMETER_UNCALIBRATED,
                                                            makeAxisRotationCalibration({
                                                                0.0f,
                                                                1.0f,
                                                                0.0f,
                                                                0.0f,
                                                                -1.0f,
                                                                0.0f,
                                                                0.0f,
                                                                0.0f,
                                                                0.0f,
                                                                0.0f,
                                                                1.0f,
                                                                0.0f,
                                                            }));
    requireVecNear(withAffineOnly, baselineUncalib, 1e-2f);
}

TEST_CASE("Test IMU streams with empty runtime calibration") {
    const auto result = runImuWithCalibration(dai::CalibrationHandler());

    REQUIRE(result.packetCount > 0);
}

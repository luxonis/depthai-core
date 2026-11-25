#include "IMU.hpp"

// std
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

// project
#include "Timesync.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/utility/CompilerWarnings.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "pipeline/NodeTiming.hpp"
#include "utilities/ClockOffset.hpp"
#include "utilities/MatrixUtils.hpp"

// libraries
#include <hardware/sensors.h>

#include "utilities/IMUDiscovery.hpp"

namespace dai {
namespace gate {

// A STRUCTURED_MUX channel carries one Android sensors_event_t record (104 bytes) per sample.
static_assert(sizeof(sensors_event_t) == 104, "sensors_event_t layout differs from the sensor hub record layout");
static_assert(offsetof(sensors_event_t, timestamp) == 16 && offsetof(sensors_event_t, flags) == 88,
              "sensors_event_t layout differs from the sensor hub record layout");

static std::string DEFAULT_IMU_FW_VERSION = "0.45.0";

std::unordered_set<int> IMU::reportIdsRequested;
spdlog::logger* IMU::pLogger = nullptr;

std::string IMU::firmwareVersion = DEFAULT_IMU_FW_VERSION;
std::string IMU::latestAvailableFirmwareVersion = DEFAULT_IMU_FW_VERSION;
std::string IMU::detectedImu = "NONE";
bool IMU::probed = false;
std::mutex IMU::imuMtx;
std::atomic<bool> IMU::instanceCount{0};

uint64_t producedGeneration = 0;
uint64_t consumedGeneration = 0;
bool readerStopped = false;
std::condition_variable cv;
// File-scope IMU state below relies on the single-instance invariant enforced by
// IMU::instanceCount in IMU::run(). If multi-instance support is added, move this
// state onto IMU instances and revisit synchronization.
static dai::IMUPacket packet;
static dai::ImuCalibrationParams imuCalibrationParams = {};
static std::vector<std::vector<float>> imuRotationMatrix = dai::kIdentityRotationMatrix;
static bool rotateAccelerometerUncalibrated = false;
static bool rotateGyroscopeUncalibrated = false;
static bool rotateMagnetometerUncalibrated = false;
static bool applyAccelerometerCalibration = false;
static bool applyGyroscopeCalibration = false;

static void applyImuRotation(float& x, float& y, float& z) {
    // Keep data unchanged when calibration contains an invalid rotation matrix.
    if(!dai::is3x3Matrix(imuRotationMatrix) || dai::isZeroRotationMatrix(imuRotationMatrix)) {
        return;
    }

    dai::apply3x3Matrix(imuRotationMatrix, x, y, z);
}

static void applyImuRotationToQuaternion(float& x, float& y, float& z, float& w) {
    if(!dai::is3x3Matrix(imuRotationMatrix) || dai::isZeroRotationMatrix(imuRotationMatrix)) {
        return;
    }

    float rx = 0.0f;
    float ry = 0.0f;
    float rz = 0.0f;
    float rw = 1.0f;
    matrixToQuaternion(imuRotationMatrix, rx, ry, rz, rw);

    float tX = 0.0f;
    float tY = 0.0f;
    float tZ = 0.0f;
    float tW = 1.0f;
    quaternionMultiply(rx, ry, rz, rw, x, y, z, w, tX, tY, tZ, tW);

    float oX = 0.0f;
    float oY = 0.0f;
    float oZ = 0.0f;
    float oW = 1.0f;
    quaternionMultiply(tX, tY, tZ, tW, -rx, -ry, -rz, rw, oX, oY, oZ, oW);

    x = oX;
    y = oY;
    z = oZ;
    w = oW;
    normalizeQuaternion(x, y, z, w);
}

dai::Timestamp getTimestamp(uint64_t qtimerNs) {
    using namespace std::chrono;
    const auto steadyDuration = duration_cast<steady_clock::duration>(nanoseconds(qtimerNs) + SteadyToQTimerOffset::instance().offset());
    const auto secs = duration_cast<seconds>(steadyDuration);

    dai::Timestamp daiTimestamp;
    daiTimestamp.sec = secs.count();
    daiTimestamp.nsec = duration_cast<nanoseconds>(steadyDuration - secs).count();
    return daiTimestamp;
}

dai::Timestamp getTimestampSystem(uint64_t nsSinceEpoch) {
    using namespace std::chrono;
    auto systemTime = system_clock::time_point(nanoseconds(nsSinceEpoch));
    dai::Timestamp daiTimestamp;
    daiTimestamp.sec = duration_cast<seconds>(systemTime.time_since_epoch()).count();
    daiTimestamp.nsec = duration_cast<nanoseconds>(systemTime.time_since_epoch()).count() % 1000000000;
    return daiTimestamp;
}

static dai::Timestamp getHostTimestamp(dai::Timestamp deviceTimestamp) {
    using namespace std::chrono;
    auto offset = Timesync::getDefaultSynchronizedOffset();
    auto tpTimestamp =
        time_point<steady_clock, steady_clock::duration>{seconds(deviceTimestamp.sec + offset.tv_sec) + nanoseconds(deviceTimestamp.nsec + offset.tv_nsec)};
    dai::Timestamp hostTimestamp;
    hostTimestamp.sec = duration_cast<seconds>(tpTimestamp.time_since_epoch()).count();
    hostTimestamp.nsec = duration_cast<nanoseconds>(tpTimestamp.time_since_epoch()).count() % 1000000000;
    return hostTimestamp;
}

static void setTimestampFromPackets(std::shared_ptr<dai::IMUData>& imuData) {
    dai::Timestamp deviceTs;
    dai::Timestamp hostTs;
    std::optional<dai::Timestamp> systemTs = std::nullopt;
    if(imuData->packets.empty()) {
        return;
    }
    for(const auto& packet : imuData->packets) {
        using namespace std::chrono;
        if(packet.acceleroMeter.tsDevice.get() > deviceTs.get()) {
            deviceTs = packet.acceleroMeter.tsDevice;
            hostTs = packet.acceleroMeter.timestamp;
        }
        if(packet.gyroscope.tsDevice.get() > deviceTs.get()) {
            deviceTs = packet.gyroscope.tsDevice;
            hostTs = packet.gyroscope.timestamp;
        }
        if(packet.magneticField.tsDevice.get() > deviceTs.get()) {
            deviceTs = packet.magneticField.tsDevice;
            hostTs = packet.magneticField.timestamp;
        }
        if(packet.rotationVector.tsDevice.get() > deviceTs.get()) {
            deviceTs = packet.rotationVector.tsDevice;
            hostTs = packet.rotationVector.timestamp;
        }

        if (packet.acceleroMeter.tsSystem.has_value()) {
            if (!systemTs.has_value()) {
                systemTs = packet.acceleroMeter.tsSystem;
            } else if (packet.acceleroMeter.tsSystem->get<system_clock>() > systemTs->get<system_clock>()) {
                systemTs = packet.acceleroMeter.tsSystem;
            }
        }
        if (packet.gyroscope.tsSystem.has_value()) {
            if (!systemTs.has_value()) {
                systemTs = packet.gyroscope.tsSystem;
            } else if (packet.gyroscope.tsSystem->get<system_clock>() > systemTs->get<system_clock>()) {
                systemTs = packet.gyroscope.tsSystem;
            }
        }
        if (packet.magneticField.tsSystem.has_value()) {
            if (!systemTs.has_value()) {
                systemTs = packet.magneticField.tsSystem;
            } else if (packet.magneticField.tsSystem->get<system_clock>() > systemTs->get<system_clock>()) {
                systemTs = packet.magneticField.tsSystem;
            }
        }
        if (packet.rotationVector.tsSystem.has_value()) {
            if (!systemTs.has_value()) {
                systemTs = packet.rotationVector.tsSystem;
            } else if (packet.rotationVector.tsSystem->get<system_clock>() > systemTs->get<system_clock>()) {
                systemTs = packet.rotationVector.tsSystem;
            }
        }

    }
    imuData->ts = hostTs;
    imuData->tsDevice = deviceTs;
    imuData->tsSystem = systemTs;
}

// Poll the shared-memory ring at least every 1 ms: sleeping a full sample period lets a newer sample overwrite the
// latest report before the consumer takes it, so the output rate drops below the request.
static useconds_t pollIntervalUs(uint32_t sampleRateHz) {
    return std::min<useconds_t>(1000, 1000000 / sampleRateHz);
}

/*
 * Read sample transmited in generic protobuf format. Used for rotv and game_rv samples.
 *
 * Producer populates IMUPacket instance with the samples received in the circular buffer from shared memory ptr between DSP and APP.
 * If no new samples are present, sleep for a short poll interval (at most 1 ms)
 */
void IMU::readGenericDataProducer(char* shared_buff_ptr, size_t sharedBuffSize, uint32_t sampleRateHz) {
    if(nullptr == shared_buff_ptr) {
        return;
    }

    SensorEventReader reader(shared_buff_ptr, sharedBuffSize);
    SensorSample sample;
    const auto pollUs = pollIntervalUs(sampleRateHz);

    /* isRunning is atomic */
    while(isRunning()) {
        const SensorEventReader::Status status = reader.next(sample);

        /* No new events (empty slot or stale one from the previous lap). Sleep until new data comes. */
        if(status == SensorEventReader::Status::NO_EVENT) {
            usleep(pollUs);
            continue;
        }

        if(status == SensorEventReader::Status::SAMPLE) {
            {
                std::lock_guard<std::mutex> dataLock(imuMtx);
                packet.rotationVector.i = sample.data[0];
                packet.rotationVector.j = sample.data[1];
                packet.rotationVector.k = sample.data[2];
                packet.rotationVector.real = sample.data[3];
                applyImuRotationToQuaternion(
                    packet.rotationVector.i, packet.rotationVector.j, packet.rotationVector.k, packet.rotationVector.real);

                packet.rotationVector.rotationVectorAccuracy = 0;
                packet.rotationVector.accuracy = dai::IMUReport::Accuracy::HIGH;

                packet.rotationVector.tsDevice = getTimestamp(sample.timestamp);
                packet.rotationVector.timestamp = getHostTimestamp(packet.rotationVector.tsDevice);
                packet.rotationVector.tsSystem = getTimestampSystem(sample.timestamp);

                ++producedGeneration;
            }
            cv.notify_one();
        }
    }

    /* If runs stop, notify all */
    {
        std::lock_guard<std::mutex> dataLock(imuMtx);
        readerStopped = true;
    }
    cv.notify_all();
}

/*
 * Read sample transmited in muxed protobuf format. Used for accel, gyro and mag samples.
 *
 * Producer populates IMUPacket instance with the samples received in the circular buffer from shared memory ptr between DSP and APP.
 * If no new samples are present, sleep for a short poll interval (at most 1 ms)
 */
void IMU::readMuxedDataProducer(char* shared_buff_ptr, size_t sharedBuffSize, uint32_t sampleRateHz) {
    if(nullptr == shared_buff_ptr) return;

    int previous_counter = 0;
    const auto pollUs = pollIntervalUs(sampleRateHz);
    char* temp = shared_buff_ptr;
    char* end_ptr = shared_buff_ptr + sharedBuffSize;

    /* isRunning is atomic */
    while(isRunning()) {
        sensors_event_t data = *reinterpret_cast<const sensors_event_t*>(temp);

        /* No events. Sleep until new events (data) come. */
        if(previous_counter >= data.reserved0) {
            usleep(pollUs);
            continue;
        }

        bool newPacket = true;
        {
            std::lock_guard<std::mutex> dataLock(imuMtx);
            switch(data.type) {
                // Never called since we are now configurating the calibration on Luxonis side instead of consuming calibrated data from Qualcomm. Keeping the case for future if we want to support consuming calibrated data from Qualcomm.
                case SENSOR_TYPE_ACCELEROMETER: {
                    float x = data.acceleration.x;
                    float y = data.acceleration.y;
                    float z = data.acceleration.z;
                    applyImuRotation(x, y, z);
                dai::apply3x4CalibrationMatrix(imuCalibrationParams.accelerometer, x, y, z);
                    packet.acceleroMeter.x = x;
                    packet.acceleroMeter.y = y;
                    packet.acceleroMeter.z = z;
                    packet.acceleroMeter.sequence = data.reserved0;

                    packet.acceleroMeter.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.acceleroMeter.tsDevice = getTimestamp(data.timestamp);
                    packet.acceleroMeter.timestamp = getHostTimestamp(packet.acceleroMeter.tsDevice);
                    packet.acceleroMeter.tsSystem = getTimestampSystem(data.timestamp);
                    packet.acceleroMeter.sequence = data.reserved0;

                    break;
                }
                // Never called since we are now configurating the calibration on Luxonis side instead of consuming calibrated data from Qualcomm. Keeping the case for future if we want to support consuming calibrated data from Qualcomm.
                case SENSOR_TYPE_MAGNETIC_FIELD:
                    packet.magneticField.x = data.magnetic.x;
                    packet.magneticField.y = data.magnetic.y;
                    packet.magneticField.z = data.magnetic.z;
                    applyImuRotation(packet.magneticField.x, packet.magneticField.y, packet.magneticField.z);

                    packet.magneticField.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.magneticField.tsDevice = getTimestamp(data.timestamp);
                    packet.magneticField.timestamp = getHostTimestamp(packet.magneticField.tsDevice);
                    packet.magneticField.tsSystem = getTimestampSystem(data.timestamp);
                    packet.magneticField.sequence = data.reserved0;

                    break;
                // Never called since we are now configurating the calibration on Luxonis side instead of consuming calibrated data from Qualcomm. Keeping the case for future if we want to support consuming calibrated data from Qualcomm.
                case SENSOR_TYPE_GYROSCOPE: {
                    float x = data.gyro.x;
                    float y = data.gyro.y;
                    float z = data.gyro.z;
                    applyImuRotation(x, y, z);
                dai::apply3x4CalibrationMatrix(imuCalibrationParams.gyroscope, x, y, z);
                    packet.gyroscope.x = x;
                    packet.gyroscope.y = y;
                    packet.gyroscope.z = z;
                    packet.gyroscope.sequence = data.reserved0;

                    packet.gyroscope.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.gyroscope.tsDevice = getTimestamp(data.timestamp);
                    packet.gyroscope.timestamp = getHostTimestamp(packet.gyroscope.tsDevice);
                    packet.gyroscope.tsSystem = getTimestampSystem(data.timestamp);
                    packet.gyroscope.sequence = data.reserved0;

                    break;
                }
                case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED: {
                    packet.acceleroMeter.x = data.uncalibrated_accelerometer.uncalib[0];
                    packet.acceleroMeter.y = data.uncalibrated_accelerometer.uncalib[1];
                    packet.acceleroMeter.z = data.uncalibrated_accelerometer.uncalib[2];
                    if(rotateAccelerometerUncalibrated) {
                        applyImuRotation(packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z);
                    }
                    if(applyAccelerometerCalibration) {
                        dai::apply3x4CalibrationMatrix(
                            imuCalibrationParams.accelerometer, packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z);
                    }

                    packet.acceleroMeter.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.acceleroMeter.tsDevice = getTimestamp(data.timestamp);
                    packet.acceleroMeter.timestamp = getHostTimestamp(packet.acceleroMeter.tsDevice);
                    packet.acceleroMeter.tsSystem = getTimestampSystem(data.timestamp);
                    packet.acceleroMeter.sequence = data.reserved0;

                    break;
                }
                case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                    packet.magneticField.x = data.uncalibrated_magnetic.uncalib[0];
                    packet.magneticField.y = data.uncalibrated_magnetic.uncalib[1];
                    packet.magneticField.z = data.uncalibrated_magnetic.uncalib[2];
                    if(rotateMagnetometerUncalibrated) {
                        applyImuRotation(packet.magneticField.x, packet.magneticField.y, packet.magneticField.z);
                    }

                    packet.magneticField.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.magneticField.tsDevice = getTimestamp(data.timestamp);
                    packet.magneticField.timestamp = getHostTimestamp(packet.magneticField.tsDevice);
                    packet.magneticField.tsSystem = getTimestampSystem(data.timestamp);
                    packet.magneticField.sequence = data.reserved0;

                    break;
                case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED: {
                    packet.gyroscope.x = data.uncalibrated_gyro.uncalib[0];
                    packet.gyroscope.y = data.uncalibrated_gyro.uncalib[1];
                    packet.gyroscope.z = data.uncalibrated_gyro.uncalib[2];
                    if(rotateGyroscopeUncalibrated) {
                        applyImuRotation(packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z);
                    }
                    if(applyGyroscopeCalibration) {
                        dai::apply3x4CalibrationMatrix(imuCalibrationParams.gyroscope, packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z);
                    }

                    packet.gyroscope.accuracy = dai::IMUReport::Accuracy::HIGH;
                    packet.gyroscope.tsDevice = getTimestamp(data.timestamp);
                    packet.gyroscope.timestamp = getHostTimestamp(packet.gyroscope.tsDevice);
                    packet.gyroscope.tsSystem = getTimestampSystem(data.timestamp);
                    packet.gyroscope.sequence = data.reserved0;

                    break;
                }
                default:
                    /* unrecognised sensor type */
                    newPacket = false;
                    break;
            }

            if(newPacket) ++producedGeneration;
        }

        if(newPacket) {
            cv.notify_one();
        }

        previous_counter = data.reserved0;
        temp = temp + sizeof(sensors_event_t);

        /* End of the circular buffer. Loop to the beginning of the buffer. */
        if(temp >= end_ptr) {
            temp = shared_buff_ptr;
        }
    }

    /* If runs stop, notify all */
    {
        std::lock_guard<std::mutex> dataLock(imuMtx);
        readerStopped = true;
    }
    cv.notify_all();
}

std::string IMU::getIMUVersion() {
    int err = Probe();
    return detectedImu;
}
std::string IMU::getIMUFirmwareVersion() {
    int err = Probe();
    return firmwareVersion;
}
std::string IMU::getEmbeddedIMUFirmwareVersion() {
    int err = Probe();
    return latestAvailableFirmwareVersion;
}
void IMU::overrideConnectedIMU(const std::string& imu) {
    probed = true;
    detectedImu = imu;
}

// Always now use the UNCALIBRATED version of the sensors from Qualcomm and apply Luxonis calibration and rotation on our side. This is to ensure the consistency of the data we consume from Qualcomm and avoid potential issue of consuming calibrated data from Qualcomm which might have gone through some unexpected transformation in Qualcomm's pipeline. The cases for calibrated sensors are kept in the code for future use if we want to support consuming calibrated data from Qualcomm.
bool IMU::daiToSensorConfig(const dai::IMUSensorConfig& src, ImuSensorConfig& dst) {
    switch(src.sensorId) {
        case dai::IMUSensor::ACCELEROMETER_RAW:
        case dai::IMUSensor::ACCELEROMETER_UNCALIBRATED:
            dst.sensorType = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "accel";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::ACCELEROMETER_CALIBRATED:
            dst.sensorType = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "accel";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
            DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
        case dai::IMUSensor::LINEAR_ACCELERATION:
        case dai::IMUSensor::GRAVITY:
            DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
            // The device does not supply these deprecated sensors. The labels stay, so that -Wswitch finds new sensors.
            return false;
        case dai::IMUSensor::GYROSCOPE_RAW:
            dst.sensorType = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "gyro";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::GYROSCOPE_CALIBRATED:
            dst.sensorType = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "gyro";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::GYROSCOPE_UNCALIBRATED:
            dst.sensorType = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "gyro";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::MAGNETOMETER_RAW:
            dst.sensorType = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "mag";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::MAGNETOMETER_CALIBRATED:
            dst.sensorType = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "mag";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::MAGNETOMETER_UNCALIBRATED:
            dst.sensorType = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
            dst.streamConfig.sensorDataType = "mag";
            dst.streamConfig.channelType = SensorChannelType::STRUCTURED_MUX;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = false;
            dst.streamConfig.resampled = true;
            break;
        case dai::IMUSensor::ROTATION_VECTOR:
            dst.sensorType = SENSOR_TYPE_ROTATION_VECTOR;
            dst.streamConfig.sensorDataType = "rotv";
            dst.streamConfig.channelType = SensorChannelType::GENERIC;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = true;
            dst.streamConfig.resampled = false;
            break;
        case dai::IMUSensor::GAME_ROTATION_VECTOR:
            dst.sensorType = SENSOR_TYPE_ROTATION_VECTOR;
            dst.streamConfig.sensorDataType = "game_rv";
            dst.streamConfig.channelType = SensorChannelType::GENERIC;
            dst.streamConfig.sampleRateHz = src.reportRate;
            dst.streamConfig.calibrated = true;
            dst.streamConfig.resampled = false;
            break;
        case dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR:
        case dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR:
        case dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR:
            return false;
        default:
            /* Sensor is not supported */
            return false;
    }

    return true;
}

int IMU::Probe() {
    std::unique_lock<std::mutex> probeLock(imuMtx);

    if(probed) {
        return 0;
    }
    probed = true;

    detectedImu = "NONE";

    auto imuDescription = dai::gate::IMUDiscovery::getInstance().getIMUDescription();

    if(imuDescription.imuType != "NONE") {
        detectedImu = imuDescription.imuType;
        latestAvailableFirmwareVersion = firmwareVersion = DEFAULT_IMU_FW_VERSION;
    }

    return 0;
}

void IMU::buildStage1() {
    pLogger = logger.get();
}

void IMU::run() {
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    if(instanceCount.exchange(true)) {
        throw std::runtime_error("There can be at most 1 instance of IMU node!");
    }

    auto imuDescription = dai::gate::IMUDiscovery::getInstance().getIMUDescription();
    imuCalibrationParams = getParentPipeline().getCalibrationData().getEepromData().imuCalibrationParams;
    imuRotationMatrix = getParentPipeline().getCalibrationData().getEepromData().imuExtrinsics.rotationMatrix;
    if(!dai::is3x3Matrix(imuRotationMatrix) || dai::isZeroRotationMatrix(imuRotationMatrix)) {
        imuRotationMatrix = dai::kIdentityRotationMatrix;
    }
    rotateAccelerometerUncalibrated = false;
    rotateGyroscopeUncalibrated = false;
    rotateMagnetometerUncalibrated = false;
    applyAccelerometerCalibration = false;
    applyGyroscopeCalibration = false;

    if(imuDescription.imuType == "NONE") {
        throw std::runtime_error("IMU not detected. Your board doesn't have IMU.");
    }

    int err = 0;
    initialReports.clear();
    reportIdsRequested.clear();

    bool mockInEnabled = mockIn.isConnected();

    int batchReportThreshold = properties.batchReportThreshold;
    int maxBatchReports = std::max(properties.batchReportThreshold, properties.maxBatchReports);
    std::vector<std::thread> activeProducerThreads;
    auto imuDataOut = std::make_shared<dai::IMUData>();
    bool accelerometerEnabled = false;
    bool gyroscopeEnabled = false;
    bool magnetometerEnabled = false;
    bool rotationVectorEnabled = false;

    if(!mockInEnabled) {
        err = Probe();
        if(err) {
            throw std::runtime_error(fmt::format("IMU not detected. Your board doesn't have IMU. {}, {}", err, __LINE__));
        }

        for(auto daicfg : properties.imuSensors) {
            if(daicfg.sensorId == dai::IMUSensor::ACCELEROMETER_CALIBRATED) {
                rotateAccelerometerUncalibrated = true;
                applyAccelerometerCalibration = true;
            } else if(daicfg.sensorId == dai::IMUSensor::ACCELEROMETER_UNCALIBRATED) {
                rotateAccelerometerUncalibrated = true;
            } else if(daicfg.sensorId == dai::IMUSensor::GYROSCOPE_CALIBRATED) {
                rotateGyroscopeUncalibrated = true;
                applyGyroscopeCalibration = true;
            } else if(daicfg.sensorId == dai::IMUSensor::GYROSCOPE_UNCALIBRATED) {
                rotateGyroscopeUncalibrated = true;
            } else if(daicfg.sensorId == dai::IMUSensor::MAGNETOMETER_CALIBRATED) {
                rotateMagnetometerUncalibrated = true;
            } else if(daicfg.sensorId == dai::IMUSensor::MAGNETOMETER_UNCALIBRATED) {
                rotateMagnetometerUncalibrated = true;
            }
            ImuSensorConfig sensorConfig;
            bool status = daiToSensorConfig(daicfg, sensorConfig);

            if(!status) {
                logger->warn("'{}' not supported.", std::to_string(static_cast<int>(daicfg.sensorId)));
                continue;
            }

            sensorConfig.sensorDirectChannel.lookupSensor(sensorConfig.streamConfig);
            if(sensorConfig.sensorDirectChannel.sensorsFound() == 0) {
                logger->warn("IMU sensor not detected. Your board does not requested sensor type {}", static_cast<int>(daicfg.sensorId));
                continue;
            }

            if(reportIdsRequested.count(sensorConfig.sensorType) != 0) {
                logger->warn("'{}' was requested multiple times, only the first config is applied.", sensorConfig.sensorType);
                continue;
            }
            reportIdsRequested.insert(sensorConfig.sensorType);
            initialReports.push_back(std::move(sensorConfig));

            switch(initialReports.back().sensorType) {
                case SENSOR_TYPE_ACCELEROMETER:
                case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
                    accelerometerEnabled = true;
                    break;
                case SENSOR_TYPE_GYROSCOPE:
                case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                    gyroscopeEnabled = true;
                    break;
                case SENSOR_TYPE_MAGNETIC_FIELD:
                case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                    magnetometerEnabled = true;
                    break;
                case SENSOR_TYPE_ROTATION_VECTOR:
                    rotationVectorEnabled = true;
                    break;
                default:
                    break;
            }
        }

        if(initialReports.size() == 0) {
            throw std::runtime_error("You didn't enable any sensor reports for IMU.");
        }

        /* Configure requested sensors */
        for(auto i = 0; i < initialReports.size(); i++) {
            int ret = initialReports.at(i).sensorDirectChannel.queryAttributes(initialReports.at(i).streamConfig);
            if(ret) {
                logger->warn("Failed to get sensor {} attributes", initialReports.at(i).streamConfig.sensorDataType);
                continue;
            }
            initialReports.at(i).sensorDirectChannel.adjustSampleRate(initialReports.at(i).streamConfig.sampleRateHz,
                                                                      initialReports.at(i).streamConfig.resampled);

            if(initialReports.at(i).streamConfig.sampleRateHz > initialReports.at(i).sensorDirectChannel.supportedSampleRates().back()) {
                logger->warn("Max sample rate for {} sensor is: {} Hz. Configured to max sample rate available.",
                             initialReports.at(i).streamConfig.sensorDataType,
                             initialReports.at(i).sensorDirectChannel.supportedSampleRates().back());
            } else if(initialReports.at(i).streamConfig.resampled) {
                // Qualcomm rounds a non-native rate up to the next supported one.
                const auto& rates = initialReports.at(i).sensorDirectChannel.supportedSampleRates();
                const auto requested = static_cast<int>(initialReports.at(i).streamConfig.sampleRateHz);
                const auto native = std::find_if(rates.begin(), rates.end(), [requested](int rate) { return rate >= requested; });
                if(native != rates.end() && *native != requested) {
                    logger->info("Requested {} Hz for {} sensor is not natively supported, sampling at {} Hz.",
                                 requested,
                                 initialReports.at(i).streamConfig.sensorDataType,
                                 *native);
                }
            }
        }

        /* Start streaming */
        for(auto i = 0; i < initialReports.size(); i++) {
            char* shared_buff_ptr;
            shared_buff_ptr = initialReports.at(i).sensorDirectChannel.startStreaming(initialReports.at(i).streamConfig);
            /* Consider to report as a hard error and stop the execution */
            if(NULL == shared_buff_ptr) {
                logger->warn("Failed to start sensor {} streaming", initialReports.at(i).streamConfig.sensorDataType);
                continue;
            }

            /* Clear communication buffer mem channel before reading */
            const size_t sharedBufferSize = SensorDirectChannel::sharedBufferSize();
            memset(shared_buff_ptr, 0, sharedBufferSize);

            /*
             * Start stream reader thread based on data format.
             *  - accel, gyro and mag streams are in muxed format
             *  - rotv stream is in generic data format
             */
            std::thread producerThread =
                (initialReports.at(i).streamConfig.channelType == SensorChannelType::GENERIC)
                    ? std::thread(&IMU::readGenericDataProducer, this, shared_buff_ptr, sharedBufferSize, initialReports.at(i).streamConfig.sampleRateHz)
                    : std::thread(&IMU::readMuxedDataProducer, this, shared_buff_ptr, sharedBufferSize, initialReports.at(i).streamConfig.sampleRateHz);

            activeProducerThreads.push_back(std::move(producerThread));
        }
    }

    /*
     * Qualcomm rounds a non-native rate up (e.g. 100 Hz -> ~116 Hz on OAK4), so cap the output to the
     * slowest requested rate. Each emit advances the deadline by one period, which keeps the mean rate
     * at the request; a packet is accepted slightly early (10% of a period) to tolerate timestamp jitter
     * when the native rate already matches. Gaps longer than a period reset the deadline (no catch-up).
     */
    const auto slowestReport = std::min_element(
        initialReports.begin(), initialReports.end(), [](const auto& a, const auto& b) { return a.streamConfig.sampleRateHz < b.streamConfig.sampleRateHz; });
    // No reports with mock input, which bypasses the decimation
    const nanoseconds outputPeriod =
        slowestReport == initialReports.end() ? nanoseconds(0) : nanoseconds(1000000000ULL / slowestReport->streamConfig.sampleRateHz);
    steady_clock::time_point nextOutputTs{};

    dai::IMUPacket lastEmittedPacket;
    unsigned int sampleCnt = 0;
    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        if(mockInEnabled) {
            std::shared_ptr<dai::IMUData> inPackets = nullptr;
            {
                auto blockEvent = this->inputBlockEvent();
                inPackets = mockIn.get<dai::IMUData>();
            }
            auto tGotInput = steady_clock::now();
            if(!inPackets) {
                throw std::runtime_error("Received message is not IMUData");
            }
            auto tProcessed = steady_clock::now();
            {
                auto blockEvent = this->outputBlockEvent();
                out.send(inPackets);
            }
            auto tAbsoluteEnd = steady_clock::now();
            dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
        } else {
            /*
             * IMU consumer thread.
             * The only one IMU instance is present, all samples have to be populated in the same packet.
             * The consumer takes populated packet from configured sensor producers.
             *
             * Every producer signals on each new sample. A packet is emitted once every enabled report has advanced since the
             * last emitted packet, so the output follows the slowest sensor and never repeats a stale report.
             * The output is then decimated to the slowest requested rate (see outputPeriod).
             */

            {
                std::unique_lock<std::mutex> dataLock(imuMtx);
                cv.wait(dataLock, []() { return readerStopped || consumedGeneration != producedGeneration; });
                consumedGeneration = producedGeneration;

                const bool allEnabledReportsAdvanced =
                    (!accelerometerEnabled || packet.acceleroMeter.tsDevice.get() > lastEmittedPacket.acceleroMeter.tsDevice.get())
                    && (!gyroscopeEnabled || packet.gyroscope.tsDevice.get() > lastEmittedPacket.gyroscope.tsDevice.get())
                    && (!magnetometerEnabled || packet.magneticField.tsDevice.get() > lastEmittedPacket.magneticField.tsDevice.get())
                    && (!rotationVectorEnabled || packet.rotationVector.tsDevice.get() > lastEmittedPacket.rotationVector.tsDevice.get());
                if(!allEnabledReportsAdvanced) {
                    continue;
                }
                const auto packetTs = std::max({accelerometerEnabled ? packet.acceleroMeter.tsDevice.get() : steady_clock::time_point{},
                                                gyroscopeEnabled ? packet.gyroscope.tsDevice.get() : steady_clock::time_point{},
                                                magnetometerEnabled ? packet.magneticField.tsDevice.get() : steady_clock::time_point{},
                                                rotationVectorEnabled ? packet.rotationVector.tsDevice.get() : steady_clock::time_point{}});
                if(packetTs + outputPeriod / 10 < nextOutputTs) {
                    continue;
                }
                nextOutputTs = packetTs >= nextOutputTs + outputPeriod ? packetTs + outputPeriod : nextOutputTs + outputPeriod;
                lastEmittedPacket = packet;

                imuDataOut->packets.push_back(packet);
                ++sampleCnt;
            }
            auto tGotInput = steady_clock::now();
            if(sampleCnt >= maxBatchReports) {
                /*
                 * Closed connection scenario.
                 * Connection may terminate during wait.
                 */
                if(!isRunning()) {
                    break;
                }
                setTimestampFromPackets(imuDataOut);
                auto tProcessed = steady_clock::now();
                {
                    auto blockEvent = this->outputBlockEvent();
                    out.send(imuDataOut);
                }
                auto tAbsoluteEnd = steady_clock::now();
                dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);

                imuDataOut = std::make_shared<dai::IMUData>();
                sampleCnt = 0;
            } else if(sampleCnt >= batchReportThreshold) {
                /*
                 * Closed connection scenario.
                 * Connection may terminate during wait.
                 */
                if(!isRunning()) {
                    break;
                }
                setTimestampFromPackets(imuDataOut);
                auto tProcessed = steady_clock::now();
                bool sent = false;
                {
                    auto blockEvent = this->outputBlockEvent();
                    sent = out.trySend(imuDataOut);
                }
                auto tAbsoluteEnd = steady_clock::now();
                dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);

                if(!sent) {
                    logger->warn("Host is reading IMU packets too slowly, increase batchReportThreshold");
                } else {
                    imuDataOut = std::make_shared<dai::IMUData>();
                    sampleCnt = 0;
                }
            }
        }
    }

    if(!mockInEnabled) {
        /* Stop producer threads and clear sensor communication channels */
        for(auto i = 0; i < initialReports.size(); i++) {
            activeProducerThreads.at(i).join();
            initialReports.at(i).sensorDirectChannel.stopStreaming(initialReports.at(i).streamConfig.calibrated, initialReports.at(i).streamConfig.resampled);
            initialReports.at(i).sensorDirectChannel.cleanup();
        }
    }
}

}  // namespace gate
}  // namespace dai

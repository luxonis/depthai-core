#pragma once

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Available IMU sensors.
 * More details about each sensor can be found in the datasheet:
 *
 * https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf
 */
enum class IMUSensor : std::int32_t {
    /**
     * Section 2.1.1
     *
     * Raw accelerometer measurement in the sensor-native frame.
     * No IMU extrinsics or affine calibration are applied.
     * Units are [m/s^2]
     *
     * @note Prior firmware versions incorrectly delivered frame-aligned data on this stream
     * (equivalent to what is now ACCELEROMETER_UNCALIBRATED). This stream now correctly
     * provides the unprocessed sensor output.
     */
    ACCELEROMETER_RAW = 0x14,
    /**
     * DepthAI synthetic accelerometer stream.
     *
     * Acceleration of the device including gravity, aligned to the DepthAI IMU frame
     * without the stored affine calibration applied.
     * Units are [m/s^2]
     */
    ACCELEROMETER_UNCALIBRATED = 0x100,
    /**
     * Section 2.1.1
     *
     * Acceleration of the device including gravity, aligned to the DepthAI IMU frame
     * and corrected with the stored affine calibration.
     * Units are [m/s^2]
     */
    ACCELEROMETER_CALIBRATED = 0x01,
    /** @deprecated Use ACCELEROMETER_CALIBRATED */
    ACCELEROMETER [[deprecated("Use ACCELEROMETER_CALIBRATED")]] = ACCELEROMETER_CALIBRATED,
    /**
     * Section 2.1.1
     *
     * Acceleration of the device with gravity removed.
     * Units are [m/s^2]
     *
     * @deprecated The affine calibration (bias) stored for the accelerometer is calibrated
     * against the full gravity-included signal and is therefore incorrect when applied to
     * this gravity-stripped output. This sensor is not correctly calibrated and may be removed
     * in a future release.
     */
    LINEAR_ACCELERATION [[deprecated("LINEAR_ACCELERATION calibration is incorrect; use ACCELEROMETER_CALIBRATED and subtract gravity on the host")]] = 0x04,
    /**
     * Section 2.1.1
     *
     * Gravity.
     * Units are [m/s^2]
     *
     * @deprecated The affine calibration (bias) stored for the accelerometer is calibrated
     * against the full gravity-included signal and is therefore incorrect when applied to
     * this gravity-only output. This sensor is not correctly calibrated and may be removed
     * in a future release.
     */
    GRAVITY [[deprecated("GRAVITY calibration is incorrect; use ACCELEROMETER_CALIBRATED and estimate gravity on the host")]] = 0x06,
    /**
     * Section 2.1.2
     *
     * Raw gyroscope measurement in the sensor-native frame.
     * DepthAI does not apply IMU extrinsics or affine calibration on this stream.
     * Units are [rad/s]
     */
    GYROSCOPE_RAW = 0x15,
    /**
     * Section 2.1.2
     *
     * Angular velocity aligned to the DepthAI IMU frame and corrected with the stored
     * affine calibration.
     * Units are [rad/s]
     */
    GYROSCOPE_CALIBRATED = 0x02,
    /**
     * Section 2.1.2
     *
     * Angular velocity aligned to the DepthAI IMU frame without the stored affine
     * calibration applied.
     * Units are [rad/s]
     */
    GYROSCOPE_UNCALIBRATED = 0x07,
    /**
     * Section 2.1.3
     *
     * Raw magnetometer measurement in the sensor-native frame.
     * DepthAI does not apply IMU extrinsics on this stream.
     * Units are [uTesla]
     */
    MAGNETOMETER_RAW = 0x16,
    /**
     * Section 2.1.3
     *
     * Magnetic field measurement aligned to the DepthAI IMU frame.
     * Units are [uTesla]
     */
    MAGNETOMETER_CALIBRATED = 0x03,
    /**
     * Section 2.1.3
     *
     * Magnetic field measurement aligned to the DepthAI IMU frame without hard-iron
     * offset applied.
     * Units are [uTesla]
     */
    MAGNETOMETER_UNCALIBRATED = 0x0f,
    /**
     * Section 2.2
     *
     * The rotation vector provides an orientation output that is expressed as a quaternion referenced to magnetic north
     * and gravity. It is produced by fusing the outputs of the accelerometer, gyroscope and magnetometer. The rotation
     * vector is the most accurate orientation estimate available. The magnetometer provides correction in yaw to
     * reduce drift and the gyroscope enables the most responsive performance.
     */
    ROTATION_VECTOR = 0x05,
    /**
     * Section 2.2
     *
     * The game rotation vector is an orientation output that is expressed as a quaternion with no specific reference for
     * heading, while roll and pitch are referenced against gravity. It is produced by fusing the outputs of the
     * accelerometer and the gyroscope (i.e. no magnetometer). The game rotation vector does not use the
     * magnetometer to correct the gyroscopes drift in yaw. This is a deliberate omission (as specified by Google) to
     * allow gaming applications to use a smoother representation of the orientation without the jumps that an
     * instantaneous correction provided by a magnetic field update could provide. Long term the output will likely drift in
     * yaw due to the characteristics of gyroscopes, but this is seen as preferable for this output versus a corrected output.
     */
    GAME_ROTATION_VECTOR = 0x08,
    /**
     * Section 2.2
     *
     * The geomagnetic rotation vector is an orientation output that is expressed as a quaternion referenced to magnetic
     * north and gravity. It is produced by fusing the outputs of the accelerometer and magnetometer. The gyroscope is
     * specifically excluded in order to produce a rotation vector output using less power than is required to produce the
     * rotation vector of section 2.2.4. The consequences of removing the gyroscope are:
     * Less responsive output since the highly dynamic outputs of the gyroscope are not used
     * More errors in the presence of varying magnetic fields.
     */
    GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    /**
     * Section 2.2
     *
     * Estimates of the magnetic field and the roll/pitch of the device can create a potential correction in the rotation
     * vector produced. For applications (typically augmented or virtual reality applications) where a sudden jump can be
     * disturbing, the output is adjusted to prevent these jumps in a manner that takes account of the velocity of the
     * sensor system.
     */
    ARVR_STABILIZED_ROTATION_VECTOR = 0x28,
    /**
     * Section 2.2
     *
     * While the magnetometer is removed from the calculation of the game rotation vector, the accelerometer itself can
     * create a potential correction in the rotation vector produced (i.e. the estimate of gravity changes). For applications
     * (typically augmented or virtual reality applications) where a sudden jump can be disturbing, the output is adjusted
     * to prevent these jumps in a manner that takes account of the velocity of the sensor system. This process is called
     * AR/VR stabilization.
     */
    ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29,
    // GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A,
};

struct IMUSensorConfig {
    /* Sensitivity enabled */
    bool sensitivityEnabled = false;

    /* Change sensitivity - true if relative; false if absolute */
    bool sensitivityRelative = false; /**< @brief Change reports relative (vs absolute) */

    // TODO write utility function to convert float to Q point notation, sensor specific
    /* 16-bit signed fixed point integer.
     * In case of absolute sensitivity represents the value a
     * sensor output must exceed in order to trigger another input
     * report.
     * In case of relative sensitivity represents the the amount
     * by which a sensor output must change from the previous
     * input report in order to trigger another input report
     * A setting of 0 causes all reports to be sent.
     */
    uint16_t changeSensitivity = 0; /**< @brief Report-on-change threshold */

    /* Rate of reports per second. (hertz)
     * 0 means disabled
     */
    uint32_t reportRate = 100;

    IMUSensor sensorId = IMUSensor::ACCELEROMETER_CALIBRATED;
};
DEPTHAI_SERIALIZE_EXT(IMUSensorConfig, sensitivityEnabled, sensitivityRelative, changeSensitivity, reportRate, sensorId);

struct IMUProperties : PropertiesSerializable<Properties, IMUProperties> {
    /* Enabled IMU sensors */
    std::vector<IMUSensorConfig> imuSensors;
    /* Above this packet threshold data will be sent to host, if queue is not blocked */
    std::int32_t batchReportThreshold = 1;
    /* Maximum number of IMU packets in a batch. Maximum 5. */
    std::int32_t maxBatchReports = 5;
    /**
     * Whether to perform firmware update or not.
     * Default value: false.
     */
    std::optional<bool> enableFirmwareUpdate = false;

    ~IMUProperties() override;
};

DEPTHAI_SERIALIZE_EXT(IMUProperties, imuSensors, batchReportThreshold, maxBatchReports, enableFirmwareUpdate);

}  // namespace dai

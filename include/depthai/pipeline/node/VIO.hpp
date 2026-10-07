#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/VIOProperties.hpp"

namespace dai {
namespace node {

/**
 * Basalt visual-inertial odometry executed on the RVC4 CPU.
 * Requires firmware with VIO support. There is no host fallback or host Basalt dependency.
 * Uses camera intrinsics and camera-to-IMU extrinsics from pipeline calibration.
 */
class VIO : public DeviceNodeCRTP<DeviceNode, VIO, VIOProperties> {
   public:
    constexpr static const char* NAME = "VIO";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Synchronized MessageGroup containing unrectified GRAY8 images named "left" and "right".
     * Device timestamps must increase and the stereo pair must be within 5 ms.
     * Default queue is nonblocking, size 4.
     */
    Input stereo{*this, {"stereo", DEFAULT_GROUP, false, 4, {{DatatypeEnum::MessageGroup, false}}}};

    /**
     * Paired raw accelerometer (m/s^2) and gyroscope (rad/s) reports with device timestamps.
     * Configure both sensors at the frequency passed to setImuUpdateRate().
     * Default queue is blocking, size 64, to avoid silently dropping IMU samples.
     */
    Input imu{*this, {"imu", DEFAULT_GROUP, true, 64, {{DatatypeEnum::IMUData, false}}}};

    /**
     * Left-camera pose in a local FLU world, in metres and a scalar-last quaternion.
     * Preserves the corresponding left image's sequence number and all timestamps.
     * No confidence or covariance is provided; a fresh pose does not guarantee valid tracking.
     */
    Output transform{*this, {"transform", DEFAULT_GROUP, {{DatatypeEnum::TransformData, false}}}};

    /** Set the IMU sample frequency in Hz; must be positive. Default: 200 Hz. */
    VIO& setImuUpdateRate(int rate);

    /** Select nominal (true) or calibrated (false) camera-to-IMU translation. Default: true. */
    VIO& setUseSpecTranslation(bool use);

    void buildStage1() override;
};

}  // namespace node
}  // namespace dai

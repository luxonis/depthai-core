#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraInfo.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImuParameters.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/StereoRectification.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
/**
 * EepromData structure
 *
 * Contains the Calibration and Board data stored on device
 */
struct EepromData {
    uint32_t version = 7;
    std::string productName, boardCustom, boardName, boardRev, boardConf, hardwareConf, deviceName;
    std::string batchName;  /// Deprecated, not used or stored
    uint64_t batchTime{0};
    uint32_t boardOptions{0};
    std::unordered_map<CameraBoardSocket, CameraInfo> cameraData;
    StereoRectification stereoRectificationData;
    Extrinsics imuExtrinsics;
    Extrinsics housingExtrinsics;
    /**
     * Legacy field retained for compatibility with serialized calibration data.
     *
     * On RVC2, Device::flashCalibration() does not persist this field and
     * Device::readCalibration() returns it as an empty vector (an empty list in Python).
     * This field is not supported for custom user data storage on RVC2.
     */
    std::vector<uint8_t> miscellaneousData;
    bool stereoUseSpecTranslation{true};
    bool stereoEnableDistortionCorrection{false};
    CameraBoardSocket verticalCameraSocket = dai::CameraBoardSocket::AUTO;
    // IMU calibration payload serialized with EepromData via DEPTHAI_SERIALIZE_OPTIONAL_EXT;
    // includes biases, scale/shear/rotation decomposition, and IMU noise parameters.
    ImuCalibrationParams imuCalibrationParams;
};

DEPTHAI_SERIALIZE_OPTIONAL_EXT_DECL(EepromData,
                                    version,
                                    boardCustom,
                                    boardName,
                                    boardRev,
                                    boardConf,
                                    hardwareConf,
                                    productName,
                                    deviceName,
                                    batchName,
                                    batchTime,
                                    boardOptions,
                                    cameraData,
                                    stereoRectificationData,
                                    imuExtrinsics,
                                    housingExtrinsics,
                                    miscellaneousData,
                                    stereoUseSpecTranslation,
                                    stereoEnableDistortionCorrection,
                                    verticalCameraSocket,
                                    imuCalibrationParams);

}  // namespace dai

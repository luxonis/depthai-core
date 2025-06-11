#pragma once

#include "depthai/common/CameraModel.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// CameraInfo structure
struct CameraInfo {
    uint16_t width = 0, height = 0;
    uint8_t lensPosition = 0;
    std::vector<std::vector<float>> intrinsicMatrix;
    std::vector<float> distortionCoeff;
    Extrinsics extrinsics;
    float specHfovDeg = 0.0f;  // fov in deg
    CameraModel cameraType = CameraModel::Perspective;
    DEPTHAI_SERIALIZE(CameraInfo, cameraType, width, height, specHfovDeg, lensPosition, intrinsicMatrix, distortionCoeff, extrinsics);
};

}  // namespace dai
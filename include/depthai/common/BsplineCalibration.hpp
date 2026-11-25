#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Cubic splined-stereographic lens model (mrcal LENSMODEL_SPLINED_STEREOGRAPHIC, order 3).
 * intrinsics: fx, fy, cx, cy, then the (x, y) correction of each control point, x varying fastest.
 */
struct BsplineCameraCalibration {
    int nx = 0, ny = 0;
    double fovXDegrees = 0;
    std::vector<double> intrinsics;

    /**
     * Throws std::invalid_argument if the grid size, field of view or parameters are invalid.
     */
    void validate() const;

    /**
     * Project a ray in camera coordinates to a pixel, as mrcal 2.5.2 does, including its extrapolation outside
     * the grid. Call validate() once before projecting.
     */
    std::array<double, 2> project(const std::array<double, 3>& ray) const;

    DEPTHAI_SERIALIZE_DECL(BsplineCameraCalibration, nx, ny, fovXDegrees, intrinsics);
};

/**
 * B-spline stereo calibration at the native sensor resolution, horizontal stereo only. On RVC4 it is stored in
 * /data, not in the EEPROM. Rotations are row-major 3x3: rotationLeft/rotationRight map raw camera rays to the
 * rectified camera, whose intrinsics are virtualIntrinsics (fx, fy, cx, cy). The stereo geometry belongs to this
 * model and is independent of the OpenCV calibration.
 */
struct BsplineStereoCalibration {
    uint32_t version = 2;
    std::string deviceId, calibrationId;
    CameraBoardSocket leftSocket = CameraBoardSocket::AUTO, rightSocket = CameraBoardSocket::AUTO;
    int width = 0, height = 0;
    BsplineCameraCalibration left, right;
    std::vector<double> rotationLeft, rotationRight, virtualIntrinsics;
    double baselineCm = 0;
    std::vector<double> leftToRightRotation, leftToRightTranslationCm;
    // Optional: the fitted left camera in the device reference frame
    CameraBoardSocket referenceSocket = CameraBoardSocket::AUTO;
    std::vector<double> leftToReferenceRotation, leftToReferenceTranslationCm;
    // Optional: housing to fitted left camera, in cm
    std::vector<double> housingToLeftRotation, housingToLeftTranslationCm;

    /**
     * 4x4 transform from a camera of this pair to the reference camera, translation in cm.
     * Throws std::invalid_argument if the model is invalid, has no reference registration or the socket is not
     * part of the pair.
     */
    std::array<std::array<double, 4>, 4> getCameraToReference(CameraBoardSocket socket) const;

    /**
     * Throws std::invalid_argument if a field is missing, not finite or inconsistent with the others.
     */
    void validate() const;

    DEPTHAI_SERIALIZE_DECL(BsplineStereoCalibration,
                           version,
                           deviceId,
                           calibrationId,
                           leftSocket,
                           rightSocket,
                           width,
                           height,
                           left,
                           right,
                           rotationLeft,
                           rotationRight,
                           virtualIntrinsics,
                           baselineCm,
                           leftToRightRotation,
                           leftToRightTranslationCm,
                           referenceSocket,
                           leftToReferenceRotation,
                           leftToReferenceTranslationCm,
                           housingToLeftRotation,
                           housingToLeftTranslationCm);
};
}  // namespace dai

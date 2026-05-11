#pragma once

#include <array>
#include <utility>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// Extrinsics structure
struct Extrinsics {
   private:
    Point3f getTranslationInUnit(bool useSpec, LengthUnit targetUnit) const;

    bool validRotationMatrix() const;

   public:
    Extrinsics() = default;

    Extrinsics(std::vector<std::vector<float>> rotationMatrix,
               Point3f translation,
               CameraBoardSocket toCameraSocket,
               LengthUnit lengthUnit = LengthUnit::CENTIMETER);

    Extrinsics(const std::vector<std::vector<float>>& extrinsicsMatrix, CameraBoardSocket toCameraSocket, LengthUnit lengthUnit = LengthUnit::CENTIMETER);

    Extrinsics(std::array<std::array<float, 4>, 4>& extrinsicsMatrix, CameraBoardSocket toCameraSocket, LengthUnit lengthUnit = LengthUnit::CENTIMETER);

    std::vector<std::vector<float>> rotationMatrix;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket obtained through calibration
     */
    Point3f translation;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket measured through CAD design
     */
    Point3f specTranslation;

    /**
     * The destination camera socket for which these extrinsics are defined.
     */
    CameraBoardSocket toCameraSocket = CameraBoardSocket::AUTO;

    /**
     * The distance unit for the translation vector.
     */
    LengthUnit lengthUnit = LengthUnit::CENTIMETER;

    /**
     * Get the extrinsic rotation matrix in array format.
     * @return 3x3 rotation matrix as a 2D array
     */
    std::vector<std::vector<float>> getRotationMatrix() const;

    /**
     * Get the inverse extrinsic rotation matrix in array format.
     * @return 3x3 inverse rotation matrix as a 2D array
     */
    std::vector<std::vector<float>> getInverseRotationMatrix() const;

    /**
     * Get the Camera Extrinsics object to the toCameraSocket.
     * @param useSpecTranslation Set to true to force using spec translation
     * @param unit Units of the returned translation vector
     * @return 4x4 homogeneous transformation matrix
     *
     * The returned matrix has the following layout:
     * ```
     * [ r00 r01 r02 Tx ]
     * [ r10 r11 r12 Ty ]
     * [ r20 r21 r22 Tz ]
     * [  0   0   0  1 ]
     * ```
     * @note The full transformation matrix can only be obtained if both the rotation matrix and the translation vector are set.
     */
    std::array<std::array<float, 4>, 4> getTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Get the inverse of the extrinsic transformation matrix which is equal to the transformation from the toCameraSocket to the current camera socket.
     * @param useSpecTranslation Set to true to force using spec translation
     * @param unit Units of the returned translation vector
     * @return a transformationMatrix which is 4x4 in homogeneous coordinate system
     */
    std::array<std::array<float, 4>, 4> getInverseTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Set the extrinsic transformation matrix.
     * @param matrix 4x4 homogeneous transformation matrix
     * @param unit Units of the translation components Tx, Ty, and Tz
     *
     * The matrix must have the following layout:
     * ```
     * [ r00 r01 r02 Tx ]
     * [ r10 r11 r12 Ty ]
     * [ r20 r21 r22 Tz ]
     * [  0   0   0  1 ]
     * ```
     */
    void setTransformationMatrix(const std::vector<std::vector<float>>& matrix, LengthUnit unit = LengthUnit::CENTIMETER);

    /**
     * Set the extrinsic transformation matrix.
     * @param matrix 4x4 homogeneous transformation matrix
     * @param unit Units of the translation components Tx, Ty, and Tz
     *
     * The matrix must have the following layout:
     * ```
     * [ r00 r01 r02 Tx ]
     * [ r10 r11 r12 Ty ]
     * [ r20 r21 r22 Tz ]
     * [  0   0   0  1 ]
     * ```
     */
    void setTransformationMatrix(const std::array<std::array<float, 4>, 4>& matrix, LengthUnit unit = LengthUnit::CENTIMETER);

    /**
     * Set the translation vector
     * @param translationVector The translation vector to set
     * @param unit Units of the provided translation vector
     * @param useSpecTranslation Set to true to force setting spec translation
     */
    void setTranslationVector(const dai::Point3f& translationVector, LengthUnit unit = LengthUnit::CENTIMETER, bool useSpecTranslation = false);

    /**
     * Get the translation vector
     * @param unit Units of the returned translation vector
     * @return translation vector in specified units
     */
    std::vector<float> getTranslationVector(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Two Extrinsics objects are equal if their rotation matrices and translation vectors are equal (within a small epsilon).
     * @param other The other Extrinsics object to compare with
     * @param epsilon The tolerance for comparing floating-point values
     * @return true if the Extrinsics objects are equal, false otherwise
     */
    bool isEqualExtrinsics(const Extrinsics& other, float epsilon = 1e-6f) const;

    /**
     * Get the extrinsic transformation matrix from this Extrinsics to the target Extrinsics.
     * @param to The target Extrinsics to get the transformation matrix to
     * @param useSpecTranslation Set to true to force using spec translation
     * @param sourceUnit Units of the translation vector in the source Extrinsics (this). Only relevant if useSpecTranslation is false.
     * @return a transformationMatrix which is 4x4 in homogeneous coordinate system
     */
    std::array<std::array<float, 4>, 4> getExtrinsicsTransformationTo(const Extrinsics& to,
                                                                      bool useSpecTranslation = false,
                                                                      LengthUnit sourceUnit = LengthUnit::CENTIMETER) const;

    DEPTHAI_SERIALIZE_OPTIONAL(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket, lengthUnit);
};

}  // namespace dai
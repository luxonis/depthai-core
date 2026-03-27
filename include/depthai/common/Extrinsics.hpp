#pragma once

#include <array>
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

   public:
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
    std::array<std::array<float, 3>, 3> getRotationMatrix() const;
    /**
     * Get the extrinsic rotation matrix inverse in array format.
     * @return 3x3 inverse rotation matrix as a 2D array
     */
    std::array<std::array<float, 3>, 3> getInverseRotationMatrix() const;
    /**
     * Get the Camera Extrinsics object to the toCameraSocket.
     * @param useSpecTranslation Set to true to force using spec translation
     * @param unit Units of the returned translation vector
     * @return a transformationMatrix which is 4x4 in homogeneous coordinate system
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     *
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
     * @param matrix 4x4 transformation matrix
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     */
    void setTransformationMatrix(const std::vector<std::vector<float>>& matrix, LengthUnit unit = LengthUnit::CENTIMETER);
    /**
     * @param matrix 4x4 transformation matrix
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     */
    void setTransformationMatrix(const std::array<std::array<float, 4>, 4>& matrix, LengthUnit unit = LengthUnit::CENTIMETER);

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

    DEPTHAI_SERIALIZE(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket, lengthUnit);
};

}  // namespace dai

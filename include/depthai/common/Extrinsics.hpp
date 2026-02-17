#pragma once

#include <stdexcept>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/utility/matrixOps.hpp"

namespace dai {

/// Extrinsics structure
struct Extrinsics {
   private:
    static constexpr LengthUnit EEPROM_TRANSLATION_UNITS = LengthUnit::CENTIMETER;

    Point3f getTranslationInUnit(bool useSpec, LengthUnit targetUnit) const {
        Point3f translationToUse = useSpec ? specTranslation : translation;
        if(useSpec && translationToUse.x == 0.0f && translationToUse.y == 0.0f && translationToUse.z == 0.0f) {
            throw std::invalid_argument("Cannot use specTranslation since it is {0, 0, 0}.");
        }
        const float scale = getDistanceUnitScale(targetUnit, EEPROM_TRANSLATION_UNITS);
        if(scale != 1.0f) {
            translationToUse.x *= scale;
            translationToUse.y *= scale;
            translationToUse.z *= scale;
        }
        return translationToUse;
    }

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

    // it should be the camera with the lowest socket number.
    // For scenario to go from camB to camC, you would calculate the transformation matrix that goes from camB to camA, then camA to camC
    CameraBoardSocket toCameraSocket = CameraBoardSocket::AUTO;

    // this is the camera from which the extrinsics are defined
    // having both from and to camera sockets helps in verifying the extrinsics direction
    CameraBoardSocket fromCameraSocket = CameraBoardSocket::AUTO;

    LengthUnit lengthUnit = LengthUnit::CENTIMETER;

    /**
     * Get the extrinsic rotation matrix in array format. The rotation matrix is 3x3 and represents the rotation from the fromCameraSocket to the
     * toCameraSocket.
     * @return 3x3 rotation matrix as a 2D array
     */
    std::array<std::array<float, 3>, 3> getRotationMatrix() const {
        if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3 || rotationMatrix[1].size() != 3 || rotationMatrix[2].size() != 3) {
            throw std::invalid_argument("Extrinsics rotationMatrix must be 3x3 to be converted to array format.");
        }
        std::array<std::array<float, 3>, 3> matrixArray;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                matrixArray[i][j] = rotationMatrix[i][j];
            }
        }
        return matrixArray;
    }

    /**
     * Get the extrinsic rotation matrix inverse in array format. The rotation matrix is 3x3 and represents the rotation from the toCameraSocket to the
     * fromCameraSocket.
     * @return 3x3 inverse rotation matrix as a 2D array
     */
    std::array<std::array<float, 3>, 3> getInverseRotationMatrix() const {
        auto rotMatrix = getRotationMatrix();
        return matrix::getMatrixInverse(rotMatrix);
    }

    /**
     * Get the Camera Extrinsics object between the fromCameraSocket and toCameraSocket
     *  between any two cameras then the relative rotation and translation is returned by this function.
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
    std::vector<std::vector<float>> getTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const {
        if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3 || rotationMatrix[1].size() != 3 || rotationMatrix[2].size() != 3) {
            throw std::invalid_argument("Extrinsics rotationMatrix must be 3x3 to build a 4x4 transformation matrix.");
        }
        std::vector<std::vector<float>> transformMatrix(4, std::vector<float>(4, 0.0f));
        // Fill in the rotation part
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                transformMatrix[i][j] = rotationMatrix[i][j];
            }
        }

        Point3f translationToUse = getTranslationInUnit(useSpecTranslation, unit);

        transformMatrix[0][3] = translationToUse.x;
        transformMatrix[1][3] = translationToUse.y;
        transformMatrix[2][3] = translationToUse.z;
        transformMatrix[3][3] = 1.0f;
        return transformMatrix;
    }

    std::vector<std::vector<float>> getInverseTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const {
        std::vector<std::vector<float>> transformMatrix = getTransformationMatrix(useSpecTranslation, unit);
        return matrix::invertSe3Matrix4x4(transformMatrix);
    }

    /**
     * Set the Extrinsics transformation matrix between fromCameraSocket and toCameraSocket
     * @param matrix 4x4 transformation matrix
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     */
    void setTransformationMatrix(const std::vector<std::vector<float>>& matrix, LengthUnit unit = LengthUnit::CENTIMETER) {
        if(matrix.size() != 4 || matrix[0].size() != 4 || matrix[1].size() != 4 || matrix[2].size() != 4 || matrix[3].size() != 4) {
            throw std::invalid_argument("Extrinsics transformation matrix must be 4x4.");
        }
        if(matrix[3][0] != 0.0f || matrix[3][1] != 0.0f || matrix[3][2] != 0.0f || matrix[3][3] != 1.0f) {
            throw std::invalid_argument("Extrinsics transformation matrix must have last row [0 0 0 1].");
        }

        rotationMatrix = {
            {matrix[0][0], matrix[0][1], matrix[0][2]},
            {matrix[1][0], matrix[1][1], matrix[1][2]},
            {matrix[2][0], matrix[2][1], matrix[2][2]},
        };

        const float scale = getDistanceUnitScale(EEPROM_TRANSLATION_UNITS, unit);
        translation = Point3f(matrix[0][3] * scale, matrix[1][3] * scale, matrix[2][3] * scale);
    }

    /**
     * Get the translation vector
     * @param unit Units of the returned translation vector
     * @return translation vector in specified units
     */
    std::vector<float> getTranslationVector(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const {
        std::vector<float> translationVector = {0, 0, 0};
        Point3f translationToUse = getTranslationInUnit(useSpecTranslation, unit);
        translationVector[0] = translationToUse.x;
        translationVector[1] = translationToUse.y;
        translationVector[2] = translationToUse.z;
        return translationVector;
    }

    bool isEqualExtrinsics(const Extrinsics& other) const {
        if(toCameraSocket != other.toCameraSocket || fromCameraSocket != other.fromCameraSocket || lengthUnit != other.lengthUnit) {
            return false;
        }
        if(!matrix::mateq(rotationMatrix, other.rotationMatrix)) {
            return false;
        }
        const auto thisTranslation = getTranslationVector(false, LengthUnit::CENTIMETER);
        const auto otherTranslation = other.getTranslationVector(false, LengthUnit::CENTIMETER);
        for(size_t i = 0; i < 3; ++i) {
            if(std::abs(thisTranslation[i] - otherTranslation[i]) > 1e-6f) {
                return false;
            }
        }

        return true;
    }

    DEPTHAI_SERIALIZE(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket, fromCameraSocket, lengthUnit);
};

}  // namespace dai

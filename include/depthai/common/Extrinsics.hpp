#pragma once

#include <array>
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
    Point3f getTranslationInUnit(bool useSpec, LengthUnit targetUnit) const {
        Point3f translationToUse = useSpec ? specTranslation : translation;
        const float scale = getDistanceUnitScale(targetUnit, lengthUnit);
        translationToUse.x *= scale;
        translationToUse.y *= scale;
        translationToUse.z *= scale;
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
    std::array<std::array<float, 3>, 3> getRotationMatrix() const {
        return matrix::vectorMatrixToMatrix3x3(rotationMatrix);
    }

    /**
     * Get the extrinsic rotation matrix inverse in array format.
     * @return 3x3 inverse rotation matrix as a 2D array
     */
    std::array<std::array<float, 3>, 3> getInverseRotationMatrix() const {
        auto rotMatrix = getRotationMatrix();
        return matrix::getMatrixInverse(rotMatrix);
    }

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
    std::array<std::array<float, 4>, 4> getTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const {
        return matrix::createTransformationMatrix(getRotationMatrix(), getTranslationInUnit(useSpecTranslation, unit));
    }

    std::array<std::array<float, 4>, 4> getInverseTransformationMatrix(bool useSpecTranslation = false, LengthUnit unit = LengthUnit::CENTIMETER) const {
        auto transformMatrix = getTransformationMatrix(useSpecTranslation, unit);
        return matrix::invertSe3Matrix4x4(transformMatrix);
    }

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
    void setTransformationMatrix(const std::vector<std::vector<float>>& matrix, LengthUnit unit = LengthUnit::CENTIMETER) {
        if(matrix.size() != 4 || matrix[0].size() != 4 || matrix[1].size() != 4 || matrix[2].size() != 4 || matrix[3].size() != 4) {
            throw std::runtime_error("Extrinsics transformation matrix must be 4x4.");
        }
        if(matrix[3][0] != 0.0f || matrix[3][1] != 0.0f || matrix[3][2] != 0.0f || matrix[3][3] != 1.0f) {
            throw std::runtime_error("Extrinsics transformation matrix must have last row [0 0 0 1].");
        }

        rotationMatrix = matrix;
        const float scale = getDistanceUnitScale(unit, lengthUnit);
        translation = Point3f(matrix[0][3] * scale, matrix[1][3] * scale, matrix[2][3] * scale);
        lengthUnit = unit;
    }

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
    void setTransformationMatrix(const std::array<std::array<float, 4>, 4>& matrix, LengthUnit unit = LengthUnit::CENTIMETER) {
        rotationMatrix = std::vector<std::vector<float>>(3, std::vector<float>(3, 0.0f));
        for(size_t i = 0; i < 3; ++i) {
            for(size_t j = 0; j < 3; ++j) {
                rotationMatrix[i][j] = matrix[i][j];
            }
        }
        const float scale = getDistanceUnitScale(unit, lengthUnit);
        translation = Point3f(matrix[0][3] * scale, matrix[1][3] * scale, matrix[2][3] * scale);
        lengthUnit = unit;
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

    /**
     * Two Extrinsics objects are equal if their rotation matrices and translation vectors are equal (within a small epsilon).
     * @param other The other Extrinsics object to compare with
     * @param epsilon The tolerance for comparing floating-point values
     * @return true if the Extrinsics objects are equal, false otherwise
     */
    bool isEqualExtrinsics(const Extrinsics& other, float epsilon = 1e-6f) const {
        if(!matrix::mateq(rotationMatrix, other.rotationMatrix)) {
            return false;
        }
        const auto thisTranslation = getTranslationVector(false, LengthUnit::CENTIMETER);
        const auto otherTranslation = other.getTranslationVector(false, LengthUnit::CENTIMETER);
        for(size_t i = 0; i < 3; ++i) {
            if(std::abs(thisTranslation[i] - otherTranslation[i]) > epsilon) {
                return false;
            }
        }

        return true;
    }

    /**
     * Get the extrinsic transformation matrix from this Extrinsics to the target Extrinsics.
     * @param to The target Extrinsics to get the transformation matrix to
     * @param useSpecTranslation Set to true to force using spec translation
     * @param sourceUnit Units of the translation vector in the source Extrinsics (this). Only relevant if useSpecTranslation is false.
     * @return a transformationMatrix which is 4x4 in homogeneous coordinate system
     */
    std::array<std::array<float, 4>, 4> getExtrinsicsTransformationTo(const Extrinsics& to,
                                                                      const bool useSpecTranslation = false,
                                                                      const LengthUnit sourceUnit = LengthUnit::CENTIMETER) const {
        if(this->toCameraSocket != to.toCameraSocket) {
            throw std::runtime_error("Cannot get extrinsics to a transformation with a different base camera socket.");
        }

        // this -> Common
        auto thisTransformationMatrix = this->getTransformationMatrix(useSpecTranslation, sourceUnit);
        // inv(to -> Common) == Common -> to
        auto toInverseTransformationMatrix = to.getInverseTransformationMatrix(useSpecTranslation, sourceUnit);
        auto thisToTransformationMatrix = matrix::matMul(toInverseTransformationMatrix, thisTransformationMatrix);

        std::array<std::array<float, 4>, 4> transformationMatrix;
        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                transformationMatrix[i][j] = thisToTransformationMatrix[i][j];
            }
        }
        return transformationMatrix;
    }

    DEPTHAI_SERIALIZE(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket, lengthUnit);
};

}  // namespace dai

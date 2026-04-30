#include "depthai/common/Extrinsics.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <depthai/utility/matrixOps.hpp>
#include <vector>

#include "depthai/common/Point3f.hpp"

namespace dai {

Extrinsics::Extrinsics(std::vector<std::vector<float>> rotationMatrix, Point3f translation, CameraBoardSocket toCameraSocket, LengthUnit lengthUnit)
    : translation(translation), toCameraSocket(toCameraSocket), lengthUnit(lengthUnit) {
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3 || rotationMatrix[1].size() != 3 || rotationMatrix[2].size() != 3) {
        throw std::runtime_error("Rotation matrix must be 3x3.");
    }
    this->rotationMatrix = std::move(rotationMatrix);
}

Extrinsics::Extrinsics(const std::vector<std::vector<float>>& extrinsicsMatrix, CameraBoardSocket toCameraSocket, LengthUnit lengthUnit)
    : toCameraSocket(toCameraSocket) {
    setTransformationMatrix(extrinsicsMatrix, lengthUnit);
}

Extrinsics::Extrinsics(std::array<std::array<float, 4>, 4>& extrinsicsMatrix, CameraBoardSocket toCameraSocket, LengthUnit lengthUnit)
    : toCameraSocket(toCameraSocket) {
    setTransformationMatrix(extrinsicsMatrix, lengthUnit);
}

std::vector<std::vector<float>> Extrinsics::getRotationMatrix() const {
    return rotationMatrix;
}

std::vector<std::vector<float>> Extrinsics::getInverseRotationMatrix() const {
    if(!validRotationMatrix()) {
        throw std::runtime_error(
            "Cannot compute inverse of invalid extrinsics rotation matrix. Please ensure the rotation matrix is properly set and is a valid rotation matrix.");
    }
    auto inv = matrix::getMatrixInverse(matrix::vectorMatrixToMatrix3x3(rotationMatrix));
    return matrix::matrix3x3ToVectorMatrix(inv);
}

std::array<std::array<float, 4>, 4> Extrinsics::getTransformationMatrix(bool useSpecTranslation, LengthUnit unit) const {
    if(!validRotationMatrix()) {
        throw std::runtime_error(
            "The full extrinsics transformation matrix can only be obtained if both the rotation matrix and the translation vector are properly set. Please "
            "ensure that the rotation matrix is a valid rotation matrix and that the translation vector is set.");
    }
    return matrix::createTransformationMatrix(getRotationMatrix(), getTranslationInUnit(useSpecTranslation, unit));
}

std::array<std::array<float, 4>, 4> Extrinsics::getInverseTransformationMatrix(bool useSpecTranslation, LengthUnit unit) const {
    auto transformMatrix = getTransformationMatrix(useSpecTranslation, unit);
    return matrix::invertSe3Matrix4x4(transformMatrix);
}

void Extrinsics::setTransformationMatrix(const std::vector<std::vector<float>>& matrix, LengthUnit unit) {
    if(matrix.size() != 4 || matrix[0].size() != 4 || matrix[1].size() != 4 || matrix[2].size() != 4 || matrix[3].size() != 4) {
        throw std::runtime_error("Extrinsics transformation matrix must be 4x4.");
    }
    if(matrix[3][0] != 0.0f || matrix[3][1] != 0.0f || matrix[3][2] != 0.0f || matrix[3][3] != 1.0f) {
        throw std::runtime_error("Extrinsics transformation matrix must have last row [0 0 0 1].");
    }

    rotationMatrix = {
        {matrix[0][0], matrix[0][1], matrix[0][2]},
        {matrix[1][0], matrix[1][1], matrix[1][2]},
        {matrix[2][0], matrix[2][1], matrix[2][2]},
    };
    translation = Point3f(matrix[0][3], matrix[1][3], matrix[2][3]);
    lengthUnit = unit;
}

void Extrinsics::setTransformationMatrix(const std::array<std::array<float, 4>, 4>& matrix, LengthUnit unit) {
    rotationMatrix = std::vector<std::vector<float>>(3, std::vector<float>(3, 0.0f));
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            rotationMatrix[i][j] = matrix[i][j];
        }
    }
    translation = Point3f(matrix[0][3], matrix[1][3], matrix[2][3]);
    lengthUnit = unit;
}

void Extrinsics::setTranslationVector(const dai::Point3f& translationVector, LengthUnit unit, bool useSpecTranslation) {
    if(useSpecTranslation) {
        specTranslation = translationVector;
    } else {
        translation = translationVector;
    }
    lengthUnit = unit;
}

std::vector<float> Extrinsics::getTranslationVector(bool useSpecTranslation, LengthUnit unit) const {
    std::vector<float> translationVector = {0, 0, 0};
    Point3f translationToUse = getTranslationInUnit(useSpecTranslation, unit);
    translationVector[0] = translationToUse.x;
    translationVector[1] = translationToUse.y;
    translationVector[2] = translationToUse.z;
    return translationVector;
}

bool Extrinsics::isEqualExtrinsics(const Extrinsics& other, float epsilon) const {
    if(!matrix::mateq(rotationMatrix, other.rotationMatrix, epsilon)) {
        return false;
    }

    if(this->toCameraSocket != other.toCameraSocket) {
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

std::array<std::array<float, 4>, 4> Extrinsics::getExtrinsicsTransformationTo(const Extrinsics& to,
                                                                              const bool useSpecTranslation,
                                                                              const LengthUnit sourceUnit) const {
    if(this->toCameraSocket == dai::CameraBoardSocket::AUTO || to.toCameraSocket == dai::CameraBoardSocket::AUTO) {
        throw std::runtime_error(
            "Cannot get extrinsics transformation to or from an extrinsics with AUTO camera socket. Please specify the camera socket for both extrinsics.");
    }
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

Point3f Extrinsics::getTranslationInUnit(bool useSpec, LengthUnit targetUnit) const {
    Point3f translationToUse = useSpec ? specTranslation : translation;
    const float scale = getDistanceUnitScale(targetUnit, lengthUnit);
    translationToUse.x *= scale;
    translationToUse.y *= scale;
    translationToUse.z *= scale;
    return translationToUse;
}

bool Extrinsics::validRotationMatrix() const {
    return rotationMatrix.size() == 3 && rotationMatrix[0].size() == 3 && rotationMatrix[1].size() == 3 && rotationMatrix[2].size() == 3;
};

}  // namespace dai
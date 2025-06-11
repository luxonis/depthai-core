#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

namespace dai {

bool PointCloudConfig::getSparse() const {
    return sparse;
}

std::array<std::array<float, 4>, 4> PointCloudConfig::getTransformationMatrix() const {
    return transformationMatrix;
}

PointCloudConfig& PointCloudConfig::setSparse(bool enable) {
    sparse = enable;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix) {
    this->transformationMatrix = transformationMatrix;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix) {
    this->transformationMatrix = {{{transformationMatrix[0][0], transformationMatrix[0][1], transformationMatrix[0][2], 0},
                                   {transformationMatrix[1][0], transformationMatrix[1][1], transformationMatrix[1][2], 0},
                                   {transformationMatrix[2][0], transformationMatrix[2][1], transformationMatrix[2][2], 0},
                                   {0, 0, 0, 1}}};
    return *this;
}

}  // namespace dai

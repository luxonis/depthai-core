#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

namespace dai {

PointCloudConfig::~PointCloudConfig() = default;

void PointCloudConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PointCloudConfig;
}

bool PointCloudConfig::getOrganized() const {
    return organized;
}

std::array<std::array<float, 4>, 4> PointCloudConfig::getTransformationMatrix() const {
    return transformationMatrix;
}

LengthUnit PointCloudConfig::getLengthUnit() const {
    return lengthUnit;
}

PointCloudConfig& PointCloudConfig::setOrganized(bool enable) {
    organized = enable;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 4>, 4>& mat) {
    transformationMatrix = mat;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 3>, 3>& mat) {
    const std::array<std::array<float, 4>, 4> m4 = {
        {{mat[0][0], mat[0][1], mat[0][2], 0.f}, {mat[1][0], mat[1][1], mat[1][2], 0.f}, {mat[2][0], mat[2][1], mat[2][2], 0.f}, {0.f, 0.f, 0.f, 1.f}}};
    return setTransformationMatrix(m4);
}

PointCloudConfig& PointCloudConfig::setLengthUnit(LengthUnit unit) {
    lengthUnit = unit;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpec) {
    coordSystemType = CoordinateSystemType::CAMERA_SOCKET;
    targetCameraSocket = targetCamera;
    useSpecTranslation = useSpec;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpec) {
    coordSystemType = CoordinateSystemType::HOUSING;
    targetHousingCS = housingCS;
    useSpecTranslation = useSpec;
    return *this;
}

PointCloudConfig::CoordinateSystemType PointCloudConfig::getCoordinateSystemType() const {
    return coordSystemType;
}

CameraBoardSocket PointCloudConfig::getTargetCameraSocket() const {
    return targetCameraSocket;
}

HousingCoordinateSystem PointCloudConfig::getTargetHousingCS() const {
    return targetHousingCS;
}

bool PointCloudConfig::getUseSpecTranslation() const {
    return useSpecTranslation;
}

}  // namespace dai

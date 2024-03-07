#include "depthai/pipeline/datatype/PointCloudConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> PointCloudConfig::serialize() const {
    return raw;
}

PointCloudConfig::PointCloudConfig() : Buffer(std::make_shared<RawPointCloudConfig>()), cfg(*dynamic_cast<RawPointCloudConfig*>(raw.get())) {}
PointCloudConfig::PointCloudConfig(std::shared_ptr<RawPointCloudConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawPointCloudConfig*>(raw.get())) {}

dai::RawPointCloudConfig PointCloudConfig::get() const {
    return cfg;
}

bool PointCloudConfig::getSparse() const {
    return cfg.sparse;
}

std::array<std::array<float, 4>, 4> PointCloudConfig::getTransformationMatrix() const {
    return cfg.transformationMatrix;
}

PointCloudConfig& PointCloudConfig::set(dai::RawPointCloudConfig config) {
    cfg = std::move(config);
    return *this;
}

PointCloudConfig& PointCloudConfig::setSparse(bool enable) {
    cfg.sparse = enable;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix) {
    cfg.transformationMatrix = transformationMatrix;
    return *this;
}

PointCloudConfig& PointCloudConfig::setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix) {
    cfg.transformationMatrix = {{{transformationMatrix[0][0], transformationMatrix[0][1], transformationMatrix[0][2], 0},
                                 {transformationMatrix[1][0], transformationMatrix[1][1], transformationMatrix[1][2], 0},
                                 {transformationMatrix[2][0], transformationMatrix[2][1], transformationMatrix[2][2], 0},
                                 {0, 0, 0, 1}}};
    return *this;
}

}  // namespace dai

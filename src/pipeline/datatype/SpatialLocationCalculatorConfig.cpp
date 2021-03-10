#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorConfig::serialize() const {
    return raw;
}

SpatialLocationCalculatorConfig::SpatialLocationCalculatorConfig()
    : Buffer(std::make_shared<RawSpatialLocationCalculatorConfig>()), cfg(*dynamic_cast<RawSpatialLocationCalculatorConfig*>(raw.get())) {}
SpatialLocationCalculatorConfig::SpatialLocationCalculatorConfig(std::shared_ptr<RawSpatialLocationCalculatorConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawSpatialLocationCalculatorConfig*>(raw.get())) {}

void SpatialLocationCalculatorConfig::setROIs(std::vector<SpatialLocationCalculatorConfigData> rois) {
    cfg.config = rois;
}

void SpatialLocationCalculatorConfig::addROI(SpatialLocationCalculatorConfigData& roi) {
    cfg.config.push_back(roi);
}

std::vector<SpatialLocationCalculatorConfigData> SpatialLocationCalculatorConfig::getConfigData() const {
    return cfg.config;
}

}  // namespace dai

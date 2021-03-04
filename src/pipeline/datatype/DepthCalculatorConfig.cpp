#include "depthai/pipeline/datatype/DepthCalculatorConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> DepthCalculatorConfig::serialize() const {
    return raw;
}

DepthCalculatorConfig::DepthCalculatorConfig()
    : Buffer(std::make_shared<RawDepthCalculatorConfig>()), cfg(*dynamic_cast<RawDepthCalculatorConfig*>(raw.get())) {}
DepthCalculatorConfig::DepthCalculatorConfig(std::shared_ptr<RawDepthCalculatorConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawDepthCalculatorConfig*>(raw.get())) {}

void DepthCalculatorConfig::setROIs(std::vector<DepthCalculatorConfigData> rois) {
    cfg.config = rois;
}

void DepthCalculatorConfig::addROI(DepthCalculatorConfigData& roi) {
    cfg.config.push_back(roi);
}

std::vector<DepthCalculatorConfigData> DepthCalculatorConfig::getConfigData() const {
    return cfg.config;
}

}  // namespace dai

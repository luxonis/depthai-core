#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
void SpatialLocationCalculatorConfig::setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs) {
    config = ROIs;
}

void SpatialLocationCalculatorConfig::addROI(SpatialLocationCalculatorConfigData& ROI) {
    config.push_back(ROI);
}

std::vector<SpatialLocationCalculatorConfigData> SpatialLocationCalculatorConfig::getConfigData() const {
    return config;
}

}  // namespace dai

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

SpatialLocationCalculatorConfig::~SpatialLocationCalculatorConfig() = default;

void SpatialLocationCalculatorConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SpatialLocationCalculatorConfig;
}

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

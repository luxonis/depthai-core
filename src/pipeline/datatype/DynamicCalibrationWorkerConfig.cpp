#include "depthai/pipeline/datatype/DynamicCalibrationWorkerConfig.hpp"

namespace dai {

DynamicCalibrationWorkerConfig::~DynamicCalibrationWorkerConfig() = default;

void DynamicCalibrationWorkerConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

}  // namespace dai

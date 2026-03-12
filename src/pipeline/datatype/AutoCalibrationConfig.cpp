#include "depthai/pipeline/datatype/AutoCalibrationConfig.hpp"

namespace dai {

AutoCalibrationConfig::~AutoCalibrationConfig() = default;

void AutoCalibrationConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

}  // namespace dai

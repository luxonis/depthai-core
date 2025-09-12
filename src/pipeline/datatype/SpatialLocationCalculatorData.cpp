#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

#if defined(__clang__)
SpatialLocationCalculatorData::~SpatialLocationCalculatorData() = default;
#endif

std::vector<SpatialLocations> SpatialLocationCalculatorData::getSpatialLocations() const {
    return spatialLocations;
}
}  // namespace dai

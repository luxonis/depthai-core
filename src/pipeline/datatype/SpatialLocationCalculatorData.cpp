#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {
std::vector<SpatialLocations> SpatialLocationCalculatorData::getSpatialLocations() const {
    return spatialLocations;
}
}  // namespace dai

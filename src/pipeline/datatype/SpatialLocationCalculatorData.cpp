#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorData::serialize() const {
    return raw;
}

SpatialLocationCalculatorData::SpatialLocationCalculatorData()
    : Buffer(std::make_shared<RawSpatialLocations>()), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}
SpatialLocationCalculatorData::SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocations> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}

std::vector<SpatialLocations>& SpatialLocationCalculatorData::getSpatialLocations() const {
    return rawdata.spatialLocations;
}

}  // namespace dai

#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorData::serialize() const {
    return raw;
}

SpatialLocationCalculatorData::SpatialLocationCalculatorData()
    : Buffer(std::make_shared<RawSpatialLocationCalculatorData>()),
      rawdata(*dynamic_cast<RawSpatialLocationCalculatorData*>(raw.get())),
      depth(rawdata.depth) {}
SpatialLocationCalculatorData::SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocationCalculatorData> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawSpatialLocationCalculatorData*>(raw.get())), depth(rawdata.depth) {}

std::vector<SpatialLocationCalculatorDataOut> SpatialLocationCalculatorData::getDepthData() const {
    return rawdata.depth;
}

}  // namespace dai

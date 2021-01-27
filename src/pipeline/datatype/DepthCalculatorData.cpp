#include "depthai/pipeline/datatype/DepthCalculatorData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> DepthCalculatorData::serialize() const {
    return raw;
}

DepthCalculatorData::DepthCalculatorData()
    : Buffer(std::make_shared<RawDepthCalculatorData>()), rawdata(*dynamic_cast<RawDepthCalculatorData*>(raw.get())), depth(rawdata.depth) {}
DepthCalculatorData::DepthCalculatorData(std::shared_ptr<RawDepthCalculatorData> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawDepthCalculatorData*>(raw.get())), depth(rawdata.depth) {}

std::vector<DepthCalculatorDataOut> DepthCalculatorData::getDepthData() const
{
    return rawdata.depth;
}

}  // namespace dai

#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class SpatialLocationCalculatorData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialLocationCalculatorData& rawdata;

   public:
    SpatialLocationCalculatorData();
    explicit SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocationCalculatorData> ptr);
    virtual ~SpatialLocationCalculatorData() = default;

    std::vector<SpatialLocationCalculatorDataOut> getDepthData() const;

    std::vector<SpatialLocationCalculatorDataOut>& depth;
};

}  // namespace dai

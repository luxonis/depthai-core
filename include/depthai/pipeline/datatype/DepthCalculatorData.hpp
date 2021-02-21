#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawDepthCalculatorData.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class DepthCalculatorData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawDepthCalculatorData& rawdata;

   public:
    DepthCalculatorData();
    explicit DepthCalculatorData(std::shared_ptr<RawDepthCalculatorData> ptr);
    virtual ~DepthCalculatorData() = default;

    std::vector<DepthCalculatorDataOut> getDepthData() const;

    std::vector<DepthCalculatorDataOut>& depth;
};

}  // namespace dai

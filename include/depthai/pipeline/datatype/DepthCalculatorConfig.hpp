#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawDepthCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class DepthCalculatorConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawDepthCalculatorConfig& cfg;

   public:
    DepthCalculatorConfig();
    explicit DepthCalculatorConfig(std::shared_ptr<RawDepthCalculatorConfig> ptr);
    virtual ~DepthCalculatorConfig() = default;

    void setROIs(std::vector<DepthCalculatorConfigData> rois);
    void addROI(DepthCalculatorConfigData& roi);
};

}  // namespace dai

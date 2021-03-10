#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class SpatialLocationCalculatorConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialLocationCalculatorConfig& cfg;

   public:
    SpatialLocationCalculatorConfig();
    explicit SpatialLocationCalculatorConfig(std::shared_ptr<RawSpatialLocationCalculatorConfig> ptr);
    virtual ~SpatialLocationCalculatorConfig() = default;

    void setROIs(std::vector<SpatialLocationCalculatorConfigData> rois);
    void addROI(SpatialLocationCalculatorConfigData& roi);

    std::vector<SpatialLocationCalculatorConfigData> getConfigData() const;
};

}  // namespace dai

#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * SpatialLocation configuration thresholds structure
 *
 * Contains configuration data for lower and upper threshold in depth units (millimeter by default) for ROI.
 * Values outside of threshold range will be ignored when calculating spatial coordinates from depth map.
 */
struct SpatialLocationCalculatorConfigThresholds {
    /**
     * Values less or equal than this threshold are not taken into calculation.
     */
    uint32_t lowerThreshold = 0;
    /**
     * Values greater or equal than this threshold are not taken into calculation.
     */
    uint32_t upperThreshold = 65535;
};
DEPTHAI_SERIALIZE_EXT(SpatialLocationCalculatorConfigThresholds, lowerThreshold, upperThreshold);

/**
 * SpatialLocationCalculatorAlgorithm configuration modes
 *
 * Contains calculation method used to obtain spatial locations.
 */
enum class SpatialLocationCalculatorAlgorithm : uint32_t { AVERAGE = 0, MEAN = AVERAGE, MIN, MAX, MODE, MEDIAN };

/// SpatialLocation configuration data structure
struct SpatialLocationCalculatorConfigData {
    static constexpr std::int32_t AUTO = -1;

    /**
     * Region of interest for spatial location calculation.
     */
    Rect roi;
    /**
     * Upper and lower thresholds for depth values to take into consideration.
     */
    SpatialLocationCalculatorConfigThresholds depthThresholds;
    /**
     * Calculation method used to obtain spatial locations
     * Average/mean: the average of ROI is used for calculation.
     * Min: the minimum value inside ROI is used for calculation.
     * Max: the maximum value inside ROI is used for calculation.
     * Mode: the most frequent value inside ROI is used for calculation.
     * Median: the median value inside ROI is used for calculation.
     * Default: median.
     */
    SpatialLocationCalculatorAlgorithm calculationAlgorithm = SpatialLocationCalculatorAlgorithm::MEDIAN;

    /**
     * Step size for calculation.
     * Step size 1 means that every pixel is taken into calculation, size 2 means every second etc.
     * Default value AUTO: for AVERAGE, MIN, MAX step size is 1; for MODE/MEDIAN it's 2.
     */
    std::int32_t stepSize = AUTO;
};
DEPTHAI_SERIALIZE_EXT(SpatialLocationCalculatorConfigData, roi, depthThresholds, calculationAlgorithm, stepSize);

/**
 * SpatialLocationCalculatorConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class SpatialLocationCalculatorConfig : public Buffer {
   public:
    /**
     * Construct SpatialLocationCalculatorConfig message.
     */
    SpatialLocationCalculatorConfig() = default;
    virtual ~SpatialLocationCalculatorConfig() = default;

    /**
     * Set a vector of ROIs as configuration data.
     * @param ROIs Vector of configuration parameters for ROIs (region of interests)
     */
    void setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs);
    /**
     * Add a new ROI to configuration data.
     * @param roi Configuration parameters for ROI (region of interest)
     */
    void addROI(SpatialLocationCalculatorConfigData& ROI);

    /**
     * Retrieve configuration data for SpatialLocationCalculator
     * @returns Vector of configuration parameters for ROIs (region of interests)
     */
    std::vector<SpatialLocationCalculatorConfigData> getConfigData() const;
    std::vector<SpatialLocationCalculatorConfigData> config;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SpatialLocationCalculatorConfig;
    };
    DEPTHAI_SERIALIZE(SpatialLocationCalculatorConfig, sequenceNum, ts, tsDevice, config);
};

}  // namespace dai

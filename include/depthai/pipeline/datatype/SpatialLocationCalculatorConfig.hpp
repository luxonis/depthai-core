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
 * Configuration for SpatialLocationCalculator.
 *
 * Holds global parameters and optional per-ROI entries used to compute 3D
 * spatial locations from a depth map.
 *
 * Global parameters (defaults):
 * - Lower depth threshold [mm]: 0
 * - Upper depth threshold [mm]: 65535
 * - Calculation algorithm: MEDIAN
 * - Step size: AUTO
 * - Keypoint radius [px]: 10
 * - Use keypoints for ImgDetections: false
 * - Use segmentation for ImgDetections: true
 *
 * An optional list of per-ROI configurations is available via `config`. ROI
 * settings override the corresponding global values where specified.
 *
 * @note The ROI-based API (`setROIs`, `addROI`) is scheduled for deprecation in future releases.
 * Users are encouraged to utilize the global configuration
 * methods instead.
 */
class SpatialLocationCalculatorConfig : public Buffer {
    static constexpr std::int32_t AUTO = -1;

   public:
    int32_t globalStepSize = AUTO;
    uint32_t globalLowerThreshold = 0;
    uint32_t globalUpperThreshold = 65535;
    SpatialLocationCalculatorAlgorithm globalCalculationAlgorithm = SpatialLocationCalculatorAlgorithm::MEDIAN;
    int32_t globalKeypointRadius = 10;
    bool calculateSpatialKeypoints = true;
    bool useSegmentation = true;
    std::vector<SpatialLocationCalculatorConfigData> config;

    /**
     * Construct SpatialLocationCalculatorConfig message.
     */
    SpatialLocationCalculatorConfig() = default;
    virtual ~SpatialLocationCalculatorConfig();

    /**
     * Specify additional regions of interest (ROI) to calculate their spatial coordinates. Results of ROI coordinates are available on
     SpatialLocationCalculatorData output.
     * @param ROIs Vector of configuration parameters for ROIs (region of interests)
     * @warning Will be deprecated in future releases.
     */
    void setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs);

    /**
     * Add a new region of interest (ROI) to configuration data.
     * @param roi Configuration parameters for ROI
     * @warning Will be deprecated in future releases.
     */
    void addROI(SpatialLocationCalculatorConfigData& ROI);

    /**
     * Set the lower and upper depth value thresholds to be used in the spatial calculations.
     * @param lowerThreshold Lower threshold in depth units (millimeter by default).
     * @param upperThreshold Upper threshold in depth units (millimeter by default).
     */
    void setDepthThresholds(uint32_t lowerThreshold = 0, uint32_t upperThreshold = 30000);

    /**
     * Set spatial location calculation algorithm. Possible values:
     *
     * - MEDIAN: Median of all depth values in the ROI
     * - AVERAGE: Average of all depth values in the ROI
     * - MIN: Minimum depth value in the ROI
     * - MAX: Maximum depth value in the ROI
     * - MODE: Most frequent depth value in the ROI
     */
    void setCalculationAlgorithm(SpatialLocationCalculatorAlgorithm calculationAlgorithm);

    /**
     * Set step size for spatial location calculation.
     * Step size 1 means that every pixel is taken into calculation, size 2 means every second etc.
     * for AVERAGE, MIN, MAX step size is 1; for MODE/MEDIAN it's 2.
     */
    void setStepSize(int32_t stepSize);

    /**
     * Set radius around keypoints to calculate spatial coordinates.
     * @param radius Radius in pixels.
     * @warning Only applicable to Keypoints or ImgDetections with keypoints.
     */
    void setKeypointRadius(int32_t radius);

    /**
     * If false, spatial coordinates of keypoints will not be calculated.
     * @param calculateSpatialKeypoints
     * @warning Only applicable to ImgDetections with keypoints.
     */
    void setCalculateSpatialKeypoints(bool calculateSpatialKeypoints);

    /**
     * Specify whether to consider only segmented pixels within a detection bounding box for spatial calculations.
     * @param useSegmentation
     * @warning Only applicable to ImgDetections with segmentation masks.
     */
    void setUseSegmentation(bool useSegmentation);

    /**
     * Retrieve configuration data for SpatialLocationCalculator
     * @returns Vector of configuration parameters for ROIs (region of interests)
     */
    std::vector<SpatialLocationCalculatorConfigData> getConfigData() const;

    /*
     * Retrieve the lower and upper depth value thresholds used in the spatial calculations.
     * @returns Pair of lower and upper thresholds in depth units (millimeter by default).
     */
    std::pair<int32_t, int32_t> getDepthThresholds() const;

    /*
     * Retrieve spatial location calculation algorithm.
     */
    SpatialLocationCalculatorAlgorithm getCalculationAlgorithm() const;

    /*
     * Retrieve step size for spatial location calculation.
     */
    int32_t getStepSize() const;

    /**
     * Retrieve radius around keypoints used to calculate spatial coordinates.
     */
    int32_t getKeypointRadius() const;

    /*
     * Retrieve whether keypoints are used for spatial location calculation.
     * @warning Only applicable to ImgDetections with keypoints.
     */
    bool getCalculateSpatialKeypoints() const;

    /*
     * Retrieve whether segmentation is used for spatial location calculation.
     * @warning Only applicable to ImgDetections with segmentation masks.
     */
    bool getUseSegmentation() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SpatialLocationCalculatorConfig;
    }

    DEPTHAI_SERIALIZE(SpatialLocationCalculatorConfig,
                      globalStepSize,
                      globalLowerThreshold,
                      globalUpperThreshold,
                      globalCalculationAlgorithm,
                      globalKeypointRadius,
                      calculateSpatialKeypoints,
                      useSegmentation,
                      config);
};

}  // namespace dai

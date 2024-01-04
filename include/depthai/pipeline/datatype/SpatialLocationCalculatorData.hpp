#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

/**
 * SpatialLocations structure
 *
 * Contains configuration data, average depth for the calculated ROI on depth map.
 * Together with spatial coordinates: x,y,z relative to the center of depth map.
 * Units are in depth units (millimeter by default).
 */
struct SpatialLocations {
    /**
     *  Configuration for selected ROI
     */
    SpatialLocationCalculatorConfigData config;
    /**
     *  Average of depth values inside the ROI between the specified thresholds in config.
     *  Calculated only if calculation method is set to AVERAGE or MIN oR MAX.
     */
    float depthAverage = 0.f;
    /**
     *  Most frequent of depth values inside the ROI between the specified thresholds in config.
     * Calculated only if calculation method is set to MODE.
     */
    float depthMode = 0.f;
    /**
     *  Median of depth values inside the ROI between the specified thresholds in config.
     * Calculated only if calculation method is set to MEDIAN.
     */
    float depthMedian = 0.f;
    /**
     *  Minimum of depth values inside the ROI between the specified thresholds in config.
     * Calculated only if calculation method is set to AVERAGE or MIN oR MAX.
     */
    std::uint16_t depthMin = 0;
    /**
     *  Maximum of depth values inside the ROI between the specified thresholds in config.
     * Calculated only if calculation method is set to AVERAGE or MIN oR MAX.
     */
    std::uint16_t depthMax = 0;
    /**
     *  Number of depth values used in calculations.
     */
    std::uint32_t depthAveragePixelCount = 0;
    /**
     *  Spatial coordinates - x,y,z; x,y are the relative positions of the center of ROI to the center of depth map
     */
    Point3f spatialCoordinates;
};
DEPTHAI_SERIALIZE_EXT(SpatialLocations, config, depthAverage, depthMode, depthMedian, depthMin, depthMax, depthAveragePixelCount, spatialCoordinates);

/**
 * SpatialLocationCalculatorData message. Carries spatial information (X,Y,Z) and their configuration parameters
 */
class SpatialLocationCalculatorData : public Buffer {
   public:
    /**
     * Construct SpatialLocationCalculatorData message.
     */
    SpatialLocationCalculatorData() = default;
    virtual ~SpatialLocationCalculatorData() = default;

    /**
     * Retrieve configuration data for SpatialLocationCalculatorData.
     * @returns Vector of spatial location data, carrying spatial information (X,Y,Z)
     */
    std::vector<SpatialLocations> getSpatialLocations() const;  // TODO - should we return a copy instead?
    // TODO(Morato) - make this private
    std::vector<SpatialLocations> spatialLocations;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SpatialLocationCalculatorData;
    };
    DEPTHAI_SERIALIZE(SpatialLocationCalculatorData, sequenceNum, ts, tsDevice, spatialLocations);
};

}  // namespace dai

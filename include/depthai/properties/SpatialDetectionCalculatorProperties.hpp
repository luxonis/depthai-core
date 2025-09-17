#pragma once

#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

enum class SpatialDetectionCalculatorAlgorithm : uint32_t {
    AVERAGE = 0,
    MEAN = AVERAGE,
    MIN,
    MAX,
    MODE,
    MEDIAN

};

enum class SpatialDetectionsMeasurementModes : uint32_t { DETAILED = 0, SEGMENTATION = 1, ELIPTICAL = 2, RECTANGLE = 3 };

struct SpatialDetectionCalculatorProperties : PropertiesSerializable<Properties, SpatialDetectionCalculatorProperties> {
    static constexpr std::int32_t AUTO = -1;

   public:
    uint32_t lowerThreshold = 0;
    uint32_t upperThreshold = 65535;
    SpatialDetectionCalculatorAlgorithm calculationAlgorithm = SpatialDetectionCalculatorAlgorithm::MEDIAN;
    SpatialDetectionsMeasurementModes measurementModel = SpatialDetectionsMeasurementModes::DETAILED;
    std::int32_t stepSize = AUTO;
};

DEPTHAI_SERIALIZE_EXT(SpatialDetectionCalculatorProperties, detections, lowerThreshold, upperThreshold, calculationAlgorithm, measurementModel, stepSize);

}  // namespace dai

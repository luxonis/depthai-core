#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * SpatialImgDetection structure
 *
 * Contains image detection results together with spatial location data.
 */
struct SpatialImgDetection : public ImgDetection {
    Point3f spatialCoordinates;
    SpatialLocationCalculatorConfigData boundingBoxMapping;
};

DEPTHAI_SERIALIZE_EXT(SpatialImgDetection, label, confidence, xmin, ymin, xmax, ymax, spatialCoordinates, boundingBoxMapping);

/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public Buffer {
   public:
    /**
     * Construct SpatialImgDetections message.
     */
    SpatialImgDetections() = default;
    virtual ~SpatialImgDetections() = default;

    /**
     * Detection results.
     */
    std::vector<SpatialImgDetection> detections;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SpatialImgDetections;
    };

    DEPTHAI_SERIALIZE(SpatialImgDetections, sequenceNum, ts, tsDevice, detections);
};

}  // namespace dai

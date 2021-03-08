#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * SpatialImgDetections message. Carries normalized detection results together with spatial coordinates
 */
class SpatialImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialImgDetections& dets;

   public:
    /// Construct SpatialImgDetections message
    SpatialImgDetections();
    explicit SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr);
    virtual ~SpatialImgDetections() = default;

    /// Detections
    std::vector<SpatialImgDetection>& detections;
};

}  // namespace dai

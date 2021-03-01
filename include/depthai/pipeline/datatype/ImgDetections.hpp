#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgDetections& dets;

   public:
    /// Construct ImgDetections message
    ImgDetections();
    explicit ImgDetections(std::shared_ptr<RawImgDetections> ptr);
    virtual ~ImgDetections() = default;

    /// Detections
    std::vector<ImgDetection>& detections;
};

}  // namespace dai

#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

// protected inheritance, so serialize isn't visible to users
class ImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgDetections& dets;

   public:
    ImgDetections();
    explicit ImgDetections(std::shared_ptr<RawImgDetections> ptr);
    virtual ~ImgDetections() = default;
    
    // reference
    std::vector<ImgDetection>& detections;
};

}  // namespace dai

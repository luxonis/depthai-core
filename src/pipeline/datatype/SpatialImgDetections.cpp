#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialImgDetections::serialize() const {
    return raw;
}

SpatialImgDetections::SpatialImgDetections()
    : Buffer(std::make_shared<RawSpatialImgDetections>()), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}
SpatialImgDetections::SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}

}  // namespace dai

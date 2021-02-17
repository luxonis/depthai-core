#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImgDetections::serialize() const {
    return raw;
}

ImgDetections::ImgDetections() : Buffer(std::make_shared<RawImgDetections>()), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}
ImgDetections::ImgDetections(std::shared_ptr<RawImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}

}  // namespace dai

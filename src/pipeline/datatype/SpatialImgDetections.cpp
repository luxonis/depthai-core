#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialImgDetections::serialize() const {
    return raw;
}

SpatialImgDetections::SpatialImgDetections()
    : Buffer(std::make_shared<RawSpatialImgDetections>()), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}
SpatialImgDetections::SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialImgDetections::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.ts.sec) + nanoseconds(dets.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialImgDetections::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.tsDevice.sec) + nanoseconds(dets.tsDevice.nsec)};
}
unsigned int SpatialImgDetections::getSequenceNum() const {
    return dets.sequenceNum;
}

}  // namespace dai

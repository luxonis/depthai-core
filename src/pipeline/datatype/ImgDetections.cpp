#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {

ImgDetections::Serialized ImgDetections::serialize() const {
    return {data, raw};
}

ImgDetections::ImgDetections() : Buffer(std::make_shared<RawImgDetections>()), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}
ImgDetections::ImgDetections(std::shared_ptr<RawImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}

// setters
ImgDetections& ImgDetections::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<ImgDetections&>(Buffer::setTimestamp(tp));
}
ImgDetections& ImgDetections::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<ImgDetections&>(Buffer::setTimestampDevice(tp));
}
ImgDetections& ImgDetections::setSequenceNum(int64_t sequenceNum) {
    return static_cast<ImgDetections&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai

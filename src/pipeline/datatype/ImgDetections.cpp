#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImgDetections::serialize() const {
    return raw;
}

ImgDetections::ImgDetections() : Buffer(std::make_shared<RawImgDetections>()), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}
ImgDetections::ImgDetections(std::shared_ptr<RawImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgDetections::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.ts.sec) + nanoseconds(dets.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgDetections::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.tsDevice.sec) + nanoseconds(dets.tsDevice.nsec)};
}
unsigned int ImgDetections::getSequenceNum() const {
    return dets.sequenceNum;
}

}  // namespace dai

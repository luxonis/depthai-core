#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialImgDetections::serialize() const {
    return raw;
}

SpatialImgDetections::SpatialImgDetections()
    : Buffer(std::make_shared<RawSpatialImgDetections>()), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}
SpatialImgDetections::SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}

// setters
SpatialImgDetections& SpatialImgDetections::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    dets.ts.sec = duration_cast<seconds>(ts).count();
    dets.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
SpatialImgDetections& SpatialImgDetections::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    dets.tsDevice.sec = duration_cast<seconds>(ts).count();
    dets.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
SpatialImgDetections& SpatialImgDetections::setSequenceNum(int64_t sequenceNum) {
    dets.sequenceNum = sequenceNum;
    return *this;
}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialImgDetections::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.ts.sec) + nanoseconds(dets.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialImgDetections::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.tsDevice.sec) + nanoseconds(dets.tsDevice.nsec)};
}
int64_t SpatialImgDetections::getSequenceNum() const {
    return dets.sequenceNum;
}

}  // namespace dai

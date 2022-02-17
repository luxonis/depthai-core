#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ImgDetections::serialize() const {
    return raw;
}

ImgDetections::ImgDetections() : Buffer(std::make_shared<RawImgDetections>()), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}
ImgDetections::ImgDetections(std::shared_ptr<RawImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawImgDetections*>(raw.get())), detections(dets.detections) {}

// setters
ImgDetections& ImgDetections::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    dets.ts.sec = duration_cast<seconds>(ts).count();
    dets.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
ImgDetections& ImgDetections::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    dets.tsDevice.sec = duration_cast<seconds>(ts).count();
    dets.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
ImgDetections& ImgDetections::setSequenceNum(int64_t sequenceNum) {
    dets.sequenceNum = sequenceNum;
    return *this;
}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgDetections::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.ts.sec) + nanoseconds(dets.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ImgDetections::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(dets.tsDevice.sec) + nanoseconds(dets.tsDevice.nsec)};
}
int64_t ImgDetections::getSequenceNum() const {
    return dets.sequenceNum;
}

}  // namespace dai

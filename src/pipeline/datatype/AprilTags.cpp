#include "depthai/pipeline/datatype/AprilTags.hpp"

namespace dai {

std::shared_ptr<RawBuffer> AprilTags::serialize() const {
    return raw;
}

AprilTags::AprilTags() : Buffer(std::make_shared<RawAprilTags>()), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}
AprilTags::AprilTags(std::shared_ptr<RawAprilTags> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> AprilTags::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.ts.sec) + nanoseconds(rawdata.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> AprilTags::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.tsDevice.sec) + nanoseconds(rawdata.tsDevice.nsec)};
}
int64_t AprilTags::getSequenceNum() const {
    return rawdata.sequenceNum;
}

// setters
AprilTags& AprilTags::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.ts.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
AprilTags& AprilTags::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.tsDevice.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
AprilTags& AprilTags::setSequenceNum(int64_t sequenceNum) {
    rawdata.sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai
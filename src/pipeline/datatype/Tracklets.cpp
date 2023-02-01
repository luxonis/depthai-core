#include "depthai/pipeline/datatype/Tracklets.hpp"

namespace dai {

std::shared_ptr<RawBuffer> Tracklets::serialize() const {
    return raw;
}

Tracklets::Tracklets() : Buffer(std::make_shared<RawTracklets>()), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}
Tracklets::Tracklets(std::shared_ptr<RawTracklets> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Tracklets::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.ts.sec) + nanoseconds(rawdata.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Tracklets::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.tsDevice.sec) + nanoseconds(rawdata.tsDevice.nsec)};
}
int64_t Tracklets::getSequenceNum() const {
    return rawdata.sequenceNum;
}

// setters
Tracklets& Tracklets::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.ts.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Tracklets& Tracklets::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.tsDevice.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Tracklets& Tracklets::setSequenceNum(int64_t sequenceNum) {
    rawdata.sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai

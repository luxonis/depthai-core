#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {

std::shared_ptr<RawBuffer> TrackedFeatures::serialize() const {
    return raw;
}

TrackedFeatures::TrackedFeatures()
    : Buffer(std::make_shared<RawTrackedFeatures>()), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}
TrackedFeatures::TrackedFeatures(std::shared_ptr<RawTrackedFeatures> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> TrackedFeatures::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.ts.sec) + nanoseconds(rawdata.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> TrackedFeatures::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.tsDevice.sec) + nanoseconds(rawdata.tsDevice.nsec)};
}
int64_t TrackedFeatures::getSequenceNum() const {
    return rawdata.sequenceNum;
}

// setters
TrackedFeatures& TrackedFeatures::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.ts.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
TrackedFeatures& TrackedFeatures::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.tsDevice.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
TrackedFeatures& TrackedFeatures::setSequenceNum(int64_t sequenceNum) {
    rawdata.sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai

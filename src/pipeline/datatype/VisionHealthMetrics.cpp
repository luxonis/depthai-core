#include "depthai/pipeline/datatype/VisionHealthMetrics.hpp"

namespace dai {

VisionHealthMetrics::Serialized VisionHealthMetrics::serialize() const {
    return {data, raw};
}

VisionHealthMetrics::VisionHealthMetrics()
    : Buffer(std::make_shared<RawVisionHealthMetrics>()),
      rawdata(*dynamic_cast<RawVisionHealthMetrics*>(raw.get())),
      visionHealthMetrics(rawdata.visionHealthMetrics) {}
VisionHealthMetrics::VisionHealthMetrics(std::shared_ptr<RawVisionHealthMetrics> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawVisionHealthMetrics*>(raw.get())), visionHealthMetrics(rawdata.visionHealthMetrics) {}

// setters
VisionHealthMetrics& VisionHealthMetrics::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<VisionHealthMetrics&>(Buffer::setTimestamp(tp));
}
VisionHealthMetrics& VisionHealthMetrics::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<VisionHealthMetrics&>(Buffer::setTimestampDevice(tp));
}
VisionHealthMetrics& VisionHealthMetrics::setSequenceNum(int64_t sequenceNum) {
    return static_cast<VisionHealthMetrics&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai

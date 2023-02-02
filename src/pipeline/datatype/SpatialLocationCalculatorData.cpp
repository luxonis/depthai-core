#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorData::serialize() const {
    return raw;
}

SpatialLocationCalculatorData::SpatialLocationCalculatorData()
    : Buffer(std::make_shared<RawSpatialLocations>()), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}
SpatialLocationCalculatorData::SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocations> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}

std::vector<SpatialLocations>& SpatialLocationCalculatorData::getSpatialLocations() const {
    return rawdata.spatialLocations;
}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialLocationCalculatorData::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.ts.sec) + nanoseconds(rawdata.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SpatialLocationCalculatorData::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.tsDevice.sec) + nanoseconds(rawdata.tsDevice.nsec)};
}
int64_t SpatialLocationCalculatorData::getSequenceNum() const {
    return rawdata.sequenceNum;
}

// setters
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setTimestamp(
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.ts.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setTimestampDevice(
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.tsDevice.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setSequenceNum(int64_t sequenceNum) {
    rawdata.sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai

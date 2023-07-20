#include "depthai/pipeline/datatype/OccupancyPool.hpp"

namespace dai {

OccupancyPool::Serialized OccupancyPool::serialize() const {
    return {data, raw};
}

OccupancyPool::OccupancyPool()
    : Buffer(std::make_shared<RawOccupancyPool>()), rawdata(*dynamic_cast<RawOccupancyPool*>(raw.get())), occupancyPool(rawdata.occupancyPool) {}
OccupancyPool::OccupancyPool(std::shared_ptr<RawOccupancyPool> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawOccupancyPool*>(raw.get())), occupancyPool(rawdata.occupancyPool) {}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> OccupancyPool::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.ts.sec) + nanoseconds(rawdata.ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> OccupancyPool::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(rawdata.tsDevice.sec) + nanoseconds(rawdata.tsDevice.nsec)};
}
int64_t OccupancyPool::getSequenceNum() const {
    return rawdata.sequenceNum;
}

// setters
OccupancyPool& OccupancyPool::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.ts.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
OccupancyPool& OccupancyPool::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    rawdata.tsDevice.sec = duration_cast<seconds>(ts).count();
    rawdata.ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
OccupancyPool& OccupancyPool::setSequenceNum(int64_t sequenceNum) {
    rawdata.sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai

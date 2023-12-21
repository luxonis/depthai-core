#include "depthai/pipeline/datatype/Buffer.hpp"

#include "depthai/utility/VectorMemory.hpp"

namespace dai {
Buffer::Buffer(size_t size) : Buffer() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}

span<uint8_t> Buffer::getData() {
    return data->getData();
}

span<const uint8_t> Buffer::getData() const {
    return data->getData();
}

void Buffer::setData(const std::vector<std::uint8_t>& d) {
    if(data->getMaxSize() >= d.size()) {
        // TODO(themarpe) - has to set offset as well
        memcpy(data->getData().data(), d.data(), d.size());
    } else {
        // allocate new holder
        data = std::make_shared<VectorMemory>(std::move(d));
    }
}

void Buffer::setData(std::vector<std::uint8_t>&& d) {
    // allocate new holder
    data = std::make_shared<VectorMemory>(std::move(d));
    // *mem = std::move(d);
    // data = mem;
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(ts.sec) + nanoseconds(ts.nsec)};
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(tsDevice.sec) + nanoseconds(tsDevice.nsec)};
}

void Buffer::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto tsLocal = tp.time_since_epoch();
    ts.sec = duration_cast<seconds>(tsLocal).count();
    ts.nsec = duration_cast<nanoseconds>(tsLocal).count() % 1000000000;
}
void Buffer::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    tsDevice.sec = duration_cast<seconds>(ts).count();
    tsDevice.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
}

int64_t Buffer::getSequenceNum() const {
    return sequenceNum;
}

void Buffer::setSequenceNum(int64_t sequenceNum) {
    sequenceNum = sequenceNum;
}

}  // namespace dai

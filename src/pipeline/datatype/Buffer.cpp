#include "depthai/pipeline/datatype/Buffer.hpp"

#include "depthai/utility/VectorMemory.hpp"

namespace dai {

Buffer::Serialized Buffer::serialize() const {
    return {data, raw};
}

Buffer::Buffer() : ADatatype(std::make_shared<dai::RawBuffer>()) {}
Buffer::Buffer(size_t size) : Buffer() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}
Buffer::Buffer(std::shared_ptr<dai::RawBuffer> ptr) : ADatatype(std::move(ptr)) {}

// helpers
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

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(raw->ts.sec) + nanoseconds(raw->ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(raw->tsDevice.sec) + nanoseconds(raw->tsDevice.nsec)};
}
int64_t Buffer::getSequenceNum() const {
    return raw->sequenceNum;
}

// setters
Buffer& Buffer::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    raw->ts.sec = duration_cast<seconds>(ts).count();
    raw->ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Buffer& Buffer::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    raw->tsDevice.sec = duration_cast<seconds>(ts).count();
    raw->tsDevice.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Buffer& Buffer::setSequenceNum(int64_t sequenceNum) {
    raw->sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai

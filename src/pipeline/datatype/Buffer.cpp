#include "depthai/pipeline/datatype/Buffer.hpp"
#include <chrono>

#include "depthai/utility/SharedMemory.hpp"
#include "depthai/utility/VectorMemory.hpp"

namespace dai {
Buffer::Buffer(size_t size) : Buffer() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(size);
    data = mem;
}

Buffer::Buffer(long fd) : Buffer() {
    auto mem = std::make_shared<SharedMemory>(fd);
    data = mem;
}

Buffer::Buffer(long fd, size_t size) : Buffer() {
    auto mem = std::make_shared<SharedMemory>(fd, size);
    data = mem;
}

Buffer::~Buffer() = default;

void Buffer::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
};

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

void Buffer::setData(const long fd) {
    data = std::make_shared<SharedMemory>(fd);
}

void Buffer::setData(std::vector<std::uint8_t>&& d) {
    // allocate new holder
    data = std::make_shared<VectorMemory>(std::move(d));
    // *mem = std::move(d);
    // data = mem;
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestamp() const {
    using namespace std::chrono;
    auto total = seconds(ts.sec) + nanoseconds(ts.nsec);
    auto dur = duration_cast<steady_clock::duration>(total);
    return time_point<steady_clock, steady_clock::duration>(dur);
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestampDevice() const {
    using namespace std::chrono;
    auto total = seconds(tsDevice.sec) + nanoseconds(tsDevice.nsec);
    auto dur = duration_cast<steady_clock::duration>(total);
    return time_point<steady_clock, steady_clock::duration>(dur);
}

std::optional<std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration>> Buffer::getTimestampSystem() const {
    if(!hasTsSystem) {
        return std::nullopt;
    }

    using namespace std::chrono;
    auto total = seconds(tsSystem.sec) + nanoseconds(tsSystem.nsec);
    auto dur = duration_cast<system_clock::duration>(total);
    return time_point<system_clock, system_clock::duration>(dur);
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
void Buffer::setTimestampSystem(std::optional<std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration>> tp) {
    // Set timestamp from timepoint
    if (!tp.has_value()) {
        hasTsSystem = false;
        tsSystem = {0, 0};
        return;
    }
    using namespace std::chrono;
    auto ts = tp->time_since_epoch();
    hasTsSystem = true;
    tsSystem.sec = duration_cast<seconds>(ts).count();
    tsSystem.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
}

int64_t Buffer::getSequenceNum() const {
    return sequenceNum;
}

void Buffer::setSequenceNum(int64_t sequenceNum) {
    this->sequenceNum = sequenceNum;
}

span<const uint8_t> Buffer::getRecordData() const {
    return data->getData();
}

dai::VisualizeType Buffer::getVisualizationMessage() const {
    return std::monostate{};
}

}  // namespace dai

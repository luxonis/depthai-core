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

}  // namespace dai

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

std::shared_ptr<dai::RawBuffer> Buffer::serialize() const {
    return raw;
}

Buffer::Buffer() : ADatatype(std::make_shared<dai::RawBuffer>()) {}
Buffer::Buffer(std::shared_ptr<dai::RawBuffer> ptr) : ADatatype(std::move(ptr)) {}

// helpers
std::vector<std::uint8_t>& Buffer::getData() {
    return raw->data;
}

void Buffer::setData(std::vector<std::uint8_t> data) {
    raw->data = std::move(data);
}

}  // namespace dai

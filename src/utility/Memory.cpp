#include "depthai/utility/Memory.hpp"

#include <stdexcept>

namespace dai {

Memory::~Memory() = default;

span<std::uint8_t> Memory::getDataRange(std::size_t offset, std::size_t length) {
    const auto bytes = getData();
    if(offset > bytes.size() || length > bytes.size() - offset) throw std::out_of_range("Memory range exceeds its storage");
    return bytes.subspan(offset, length);
}

span<const std::uint8_t> Memory::getDataRange(std::size_t offset, std::size_t length) const {
    const auto bytes = getData();
    if(offset > bytes.size() || length > bytes.size() - offset) throw std::out_of_range("Memory range exceeds its storage");
    return bytes.subspan(offset, length);
}

std::vector<span<const std::uint8_t>> Memory::getDataSegments() const {
    return {getData()};
}

}  // namespace dai

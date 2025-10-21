#include "depthai/utility/VectorMemory.hpp"

namespace dai {

VectorMemory::~VectorMemory() = default;

span<std::uint8_t> VectorMemory::getData() {
    return {data(), size()};
}
span<const std::uint8_t> VectorMemory::getData() const {
    return {data(), size()};
}
std::size_t VectorMemory::getMaxSize() const {
    return capacity();
}
std::size_t VectorMemory::getOffset() const {
    return 0;
}
void VectorMemory::setSize(std::size_t size) {
    resize(size);
}

}  // namespace dai
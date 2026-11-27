#pragma once

#include <cstdint>
#include <vector>

namespace dai {
namespace utility {

template <typename T>
inline std::vector<std::uint8_t> serialize(const T& obj) {
    (void)obj;
    return {};
}

}  // namespace utility
}  // namespace dai

#define DEPTHAI_SERIALIZE(...)
#define DEPTHAI_SERIALIZE_EXT(...)

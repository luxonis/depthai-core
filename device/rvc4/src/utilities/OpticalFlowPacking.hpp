#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dai {
namespace gate {
namespace optical_flow {

// Native LSB-packed signed Q8.4. Sign extend bit 11 explicitly: accepting both
// zero-padded and sign-extended storage avoids implementation-defined signed shifts.
inline float decodeMotionVector(std::int16_t storage) {
    const auto bits = static_cast<std::uint16_t>(storage) & 0x0fffU;
    const auto signedValue = static_cast<int>(bits) - ((bits & 0x0800U) ? 4096 : 0);
    return static_cast<float>(signedValue) / 16.0f;
}

inline std::vector<float> copyMotionVectors(const std::int16_t* source, std::size_t components) {
    if(!source) throw std::invalid_argument("Missing native optical flow buffer");
    std::vector<float> result(components);
    for(std::size_t i = 0; i < components; ++i) result[i] = decodeMotionVector(source[i]);
    return result;
}

}  // namespace optical_flow
}  // namespace gate
}  // namespace dai

#pragma once

#include <cstdint>
#include <vector>

namespace dai {

enum class Profile { H264, H265 };
enum class SliceType { P, B, I, SP, SI, Unknown };

std::vector<SliceType> getTypesH264(const std::vector<std::uint8_t>& bs, bool breakOnFirst = false);
std::vector<SliceType> getTypesH265(const std::vector<std::uint8_t>& bs, bool breakOnFirst = false);

}  // namespace dai

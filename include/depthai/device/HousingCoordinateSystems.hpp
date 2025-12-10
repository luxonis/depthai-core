#pragma once

#include <unordered_map>
#include <string>
#include <array>
#include <cstdint>

#include "depthai/common/HousingCoordinateSystem.hpp"

namespace dai {

/**
 * Get housing coordinate systems for all supported products.
 * Returns a map: product name -> (HousingCoordinateSystem enum -> [x, y, z] translation in mm)
 */
#ifdef DEPTHAI_HAVE_HOUSING_COORDINATES
const std::unordered_map<std::string, std::unordered_map<int32_t, std::array<float, 3>>>& getHousingCoordinateSystems();
#else
inline const std::unordered_map<std::string, std::unordered_map<int32_t, std::array<float, 3>>>& getHousingCoordinateSystems() {
    static const std::unordered_map<std::string, std::unordered_map<int32_t, std::array<float, 3>>> emptyMap;
    return emptyMap;
}
#endif

} // namespace dai
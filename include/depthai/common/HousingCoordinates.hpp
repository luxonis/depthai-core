#include <unordered_map>
#include <string>
#include <array>
#include <cstdint>

// depthai
#include "HousingCoordinateSystem.hpp"

namespace dai {

#ifdef DEPTHAI_HAVE_HOUSING_COORDINATES

const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> getHousingCoordinates();

#else

const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> getHousingCoordinates() {
  return {};
}

#endif

} // namespace dai
#include "device/HousingCoordinateSystems.hpp"

namespace dai {

static const std::unordered_map<std::string, std::unordered_map<int32_t, std::array<float, 3>>> dataMap = @HOUSING_COORDINATE_DATA@;

const std::unordered_map<std::string, std::unordered_map<int32_t, std::array<float, 3>>>& getHousingCoordinateSystems() {
    return dataMap;
}

} // namespace dai
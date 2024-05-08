#include <optional>
#include <variant>
#include <vector>

// depthai public
#include <depthai/common/optional.hpp>
#include <depthai/common/variant.hpp>
#include <depthai/utility/Serialization.hpp>

namespace dai {

template <typename T>
class CapabilityRange {
   public:
    void minMax(const std::pair<T, T>& minMax) {
        value = minMax;
    }
    void minMax(const std::tuple<T, T>& minMax) {
        value = std::make_pair(std::get<0>(minMax), std::get<1>(minMax));
    }
    void minMax(const T& min, const T& max) {
        value = std::make_pair(min, max);
    }
    void fixed(const T& fixed) {
        value = fixed;
    }
    void discrete(const std::vector<T>& discreteValues) {
        value = discreteValues;
    }

    std::optional<std::variant<T, std::pair<T, T>, std::vector<T>>> value = std::nullopt;

    DEPTHAI_SERIALIZE(CapabilityRange, value);
};

}  // namespace dai

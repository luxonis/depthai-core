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
    std::optional<std::variant<T, std::pair<T, T>, std::vector<T>>> value = std::nullopt;
    DEPTHAI_SERIALIZE(CapabilityRange, value);
};

}  // namespace dai

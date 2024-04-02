#include <optional>
#include <variant>
#include <vector>

namespace dai {

template <typename T>
class CapabilityRange {
   public:
    std::optional<std::variant<T, std::pair<T, T>, std::vector<T>>> value = std::nullopt;
};

}  // namespace dai

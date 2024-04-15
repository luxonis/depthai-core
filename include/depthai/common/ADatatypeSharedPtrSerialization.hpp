#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/Serialization.hpp"
namespace nlohmann {
template <>
struct adl_serializer<std::shared_ptr<dai::ADatatype>> {
    static void to_json(json& j, const std::shared_ptr<dai::ADatatype>& /*opt*/) {  // NOLINT this is a specialization, naming conventions don't apply
        j = nullptr;
    }

    static void from_json(const json& /*j*/, std::shared_ptr<dai::ADatatype>& opt) {  // NOLINT this is a specialization, naming conventions don't apply
        opt = nullptr;
    }
};
}  // namespace nlohmann

namespace nop {
template <>
struct Encoding<std::shared_ptr<dai::ADatatype>> : EncodingIO<std::shared_ptr<dai::ADatatype>> {
    using Type = std::shared_ptr<dai::ADatatype>;

    static constexpr EncodingByte Prefix(const Type& /* value */) {
        return EncodingByte::Empty;
    }

    static constexpr std::size_t Size(const Type& /* value */) {
        return 0;
    }

    static constexpr bool Match(EncodingByte prefix) {
        return prefix == EncodingByte::Empty;
    }

    template <typename Writer>
    static constexpr Status<void> WritePayload(EncodingByte /* prefix */, const Type& /* value */, Writer* /* writer */) {
        return {};
    }

    template <typename Reader>
    static constexpr Status<void> ReadPayload(EncodingByte /* prefix */, Type* value, Reader* /* reader */) {
        value->reset();
        return {};
    }
};
}  // namespace nop
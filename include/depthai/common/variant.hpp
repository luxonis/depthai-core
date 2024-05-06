#pragma once

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <variant>

using json = nlohmann::json;

namespace dai {
namespace internal {
template <std::size_t N>
struct VariantSwitch {
    template <typename Variant>
    void operator()(int index, json const& value, Variant& v) const {
        if(index == N)
            v = value.get<std::variant_alternative_t<N, Variant>>();
        else
            VariantSwitch<N - 1>{}(index, value, v);
    }
};

template <>
struct VariantSwitch<0> {
    template <typename Variant>
    void operator()(int index, json const& value, Variant& v) const {
        if(index == 0)
            v = value.get<std::variant_alternative_t<0, Variant>>();
        else {
            throw std::runtime_error("while converting json to variant: invalid index");
        }
    }
};
}  // namespace internal
}  // namespace dai

namespace nlohmann {
template <typename... Args>
struct adl_serializer<std::variant<Args...>> {
    static void to_json(json& j, std::variant<Args...> const& v) {  // NOLINT this is a specialization, naming conventi  ons don't apply
        std::visit(
            [&](auto&& value) {
                j["index"] = v.index();
                j["value"] = std::forward<decltype(value)>(value);
            },
            v);
    }

    static void from_json(json const& j, std::variant<Args...>& v) {  // NOLINT this is a specialization, naming conventi  ons don't apply
        auto const index = j.at("index").get<int>();
        ::dai::internal::VariantSwitch<sizeof...(Args) - 1>{}(index, j.at("value"), v);
    }
};
}  // namespace nlohmann

namespace dai {
namespace internal {
template <std::size_t N>
struct VariantReadNop {
    template <typename Variant, typename Reader>
    void operator()(int index, Reader* reader, Variant& v) const {
        if(index == N) {
            // using Element = typename std::decay<std::variant_alternative_t<N, Variant>>::type;
            using Element = typename std::variant_alternative_t<N, Variant>;
            Element element;
            const auto status = nop::Encoding<Element>::Read(&element, reader);
            if(!status) {
                throw std::runtime_error("converting libnop object to variant failed: nop::Encoding::Read failed");
            }
            v = element;
        } else {
            VariantReadNop<N - 1>{}(index, reader, v);
        }
    }
};

template <>
struct VariantReadNop<0> {
    template <typename Variant, typename Reader>
    void operator()(int index, Reader* reader, Variant& v) const {
        if(index == 0) {
            // using Element = typename std::decay<std::variant_alternative_t<0, Variant>>::type;
            using Element = typename std::variant_alternative_t<0, Variant>;
            Element element;
            const auto status = nop::Encoding<Element>::Read(&element, reader);
            if(!status) {
                throw std::runtime_error("converting libnop object to variant failed: nop::Encoding::Read failed");
            }
            v = element;
        } else {
            throw std::runtime_error("converting libnop object to variant failed: invalid index");
        }
    }
};
}  // namespace internal
}  // namespace dai

// std::variant serialization for libnop
namespace nop {
//
// std::variant<Ts...> encoding format:
//
// +-----+---------+-----------+
// | VAR | INT32:I | ELEMENT I |
// +-----+---------+-----------+
//
// Elements are expected to be valid encodings for their element type.
//
// EmptyVariant encoding format:
//
// +-----+
// | NIL |
// +-----+
//
// Therefore a Variant in the empty state has this specific encoding:
//
// +-----+----+-----+
// | VAR | -1 | NIL |
// +-----+----+-----+
//

template <typename... Ts>
struct Encoding<std::variant<Ts...>> : EncodingIO<std::variant<Ts...>> {
    using Type = std::variant<Ts...>;

    static constexpr EncodingByte Prefix(const Type& /*value*/) {
        return EncodingByte::Variant;
    }

    static constexpr std::size_t Size(const Type& value) {
        return BaseEncodingSize(Prefix(value)) + Encoding<std::int32_t>::Size(value.index())
               + std::visit(
                   [](const auto& element) {
                       using Element = typename std::decay<decltype(element)>::type;
                       return Encoding<Element>::Size(element);
                   },
                   value);
    }

    static constexpr bool Match(EncodingByte prefix) {
        return prefix == EncodingByte::Variant;
    }

    template <typename Writer>
    static constexpr Status<void> WritePayload(EncodingByte /*prefix*/, const Type& value, Writer* writer) {
        auto status = Encoding<std::int32_t>::Write(value.index(), writer);
        if(!status) return status;

        return std::visit(
            [writer](const auto& element) {
                using Element = typename std::decay<decltype(element)>::type;
                return Encoding<Element>::Write(element, writer);
            },
            value);
    }

    template <typename Reader>
    static constexpr Status<void> ReadPayload(EncodingByte /*prefix*/, Type* value, Reader* reader) {
        std::int32_t index = 0;
        auto status = Encoding<std::int32_t>::Read(&index, reader);
        if(!status) {
            return status;
        } else if(index < 0 || index >= static_cast<std::int32_t>(sizeof...(Ts))) {
            return ErrorStatus::UnexpectedVariantType;
        }
        ::dai::internal::VariantReadNop<sizeof...(Ts) - 1>{}(index, reader, *value);
        return {};
    }
};
}  // namespace nop

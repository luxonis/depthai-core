#pragma once

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <variant>

using json = nlohmann::json;

namespace detail {
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
}  // namespace detail

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
        ::detail::VariantSwitch<sizeof...(Args) - 1>{}(index, j.at("value"), v);
    }
};
}  // namespace nlohmann

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
        std::int32_t type = 0;
        auto status = Encoding<std::int32_t>::Read(&type, reader);
        if(!status) {
            return status;
        } else if(type < Type::kEmptyIndex || type >= static_cast<std::int32_t>(sizeof...(Ts))) {
            return ErrorStatus::UnexpectedVariantType;
        }
        value->Become(type);
        return std::visit(
            [reader](auto&& element) {
                using Element = typename std::decay<decltype(element)>::type;
                return Encoding<Element>::Read(&element, reader);
            },
            value);
    }
};
}  // namespace nop

#pragma once

#include "depthai/utility/Serialization.hpp"
#include <optional>

// std::optional serialization for nlohmann json
// partial specialization (full specialization works too)
namespace nlohmann {
template <typename T>
struct adl_serializer<std::optional<T>> {
    static void to_json(json& j, const std::optional<T>& opt) {  // NOLINT this is a specialization, naming conventions don't apply
        if(opt == std::nullopt) {
            j = nullptr;
        } else {
            j = *opt;  // this will call adl_serializer<T>::to_json which will
                       // find the free function to_json in T's namespace!
        }
    }

    static void from_json(const json& j, std::optional<T>& opt) {  // NOLINT this is a specialization, naming conventions don't apply
        if(j.is_null()) {
            opt = std::nullopt;
        } else {
            opt = j.get<T>();  // same as above, but with
                               // adl_serializer<T>::from_json
        }
    }
    static std::optional<T> from_json(const json& j) {
        if(j.is_null())
            return std::make_optional<T>();
        else
            return std::make_optional<T>(j.get<T>());
    }
};
}  // namespace nlohmann

// std::optional serialization for libnop
namespace nop {

//
// Optional<T> encoding formats:
//
// Empty Optional<T>:
//
// +-----+
// | NIL |
// +-----+
//
// Non-empty Optional<T>
//
// +---//----+
// | ELEMENT |
// +---//----+
//
// Element must be a valid encoding of type T.
//

template <typename T>
struct Encoding<std::optional<T>> : EncodingIO<std::optional<T>> {
    using Type = std::optional<T>;

    static constexpr EncodingByte Prefix(const Type& value) {
        return value ? Encoding<T>::Prefix(*value) : EncodingByte::Empty;
    }

    static constexpr std::size_t Size(const Type& value) {
        return value ? Encoding<T>::Size(*value) : BaseEncodingSize(EncodingByte::Empty);
    }

    static constexpr bool Match(EncodingByte prefix) {
        return prefix == EncodingByte::Empty || Encoding<T>::Match(prefix);
    }

    template <typename Writer>
    static constexpr Status<void> WritePayload(EncodingByte prefix, const Type& value, Writer* writer) {
        if(value) {
            return Encoding<T>::WritePayload(prefix, *value, writer);
        } else {
            return {};
        }
    }

    template <typename Reader>
    static constexpr Status<void> ReadPayload(EncodingByte prefix, Type* value, Reader* reader) {
        if(prefix == EncodingByte::Empty) {
            value->reset();
        } else {
            T temp;
            auto status = Encoding<T>::ReadPayload(prefix, &temp, reader);
            if(!status) return status;

            *value = std::move(temp);
        }

        return {};
    }
};

}  // namespace nop

#pragma once

#include <utility>

#include "depthai/utility/Serialization.hpp"

namespace nop {

// Store arbitrary JSON as a MessagePack binary payload inside libnop structures.
template <>
struct Encoding<nlohmann::json> : EncodingIO<nlohmann::json> {
    using Type = nlohmann::json;
    using Bytes = std::vector<std::uint8_t>;

    static constexpr EncodingByte Prefix(const Type& /*value*/) {
        return EncodingByte::Binary;
    }

    static std::size_t Size(const Type& value) {
        return Encoding<Bytes>::Size(Type::to_msgpack(value));
    }

    static constexpr bool Match(EncodingByte prefix) {
        return Encoding<Bytes>::Match(prefix);
    }

    template <typename Writer>
    static Status<void> WritePayload(EncodingByte prefix, const Type& value, Writer* writer) {
        return Encoding<Bytes>::WritePayload(prefix, Type::to_msgpack(value), writer);
    }

    template <typename Reader>
    static Status<void> ReadPayload(EncodingByte prefix, Type* value, Reader* reader) {
        Bytes bytes;
        const auto status = Encoding<Bytes>::ReadPayload(prefix, &bytes, reader);
        if(!status) return status;
        auto parsed = Type::from_msgpack(bytes, true, false);
        if(parsed.is_discarded()) return ErrorStatus::ProtocolError;
        *value = std::move(parsed);
        return {};
    }
};

}  // namespace nop

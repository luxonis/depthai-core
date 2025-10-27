#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dai {

class ProtoSerializable {
   public:
    struct SchemaPair {
        std::string schemaName;
        std::string schema;
    };

    virtual ~ProtoSerializable();

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * @brief Serialize the protobuf message of this object
     * @return serialized protobuf message
     */
    virtual std::vector<std::uint8_t> serializeProto(bool metadataOnly = false) const = 0;

    /**
     * @brief Serialize the schema of this object
     * @return schemaPair
     */
    virtual SchemaPair serializeSchema() const = 0;

#else
    // Helper struct for compile-time check
    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };

    /**
     * @brief Placeholder for serializeProto when Protobuf is disabled
     * @return Throws compile-time error if used
     */
    template <typename... T>
    std::vector<std::uint8_t> serializeProto(T...) const {
        static_assert(dependent_false<T...>::value, "Protobuf support is not enabled in this build");
        return {};
    }

    /**
     * @brief Placeholder for serializeSchema when Protobuf is disabled
     * @return Throws compile-time error if used
     */
    template <typename... T>
    SchemaPair serializeSchema(T...) const {
        static_assert(dependent_false<T...>::value, "Protobuf support is not enabled in this build");
        return {};
    }
#endif
};

}  // namespace dai

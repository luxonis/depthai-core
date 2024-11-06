#pragma once

#include <vector>
namespace dai {

namespace utility {

class ProtoSerializable {
   public:
    struct SchemaPair {
        std::string schemaName;
        std::string schema;
    };

    virtual ~ProtoSerializable() = default;

    /**
     * @brief Serialize the protobuf message of this object
     * @return serialized protobuf message
     */
    virtual std::vector<std::uint8_t> serializeProto() const = 0;

    /**
     * @brief Serialize the schema of this object
     * @return schemaPair
     */
    virtual SchemaPair serializeSchema() const = 0;
};

}  // namespace utility

}  // namespace dai
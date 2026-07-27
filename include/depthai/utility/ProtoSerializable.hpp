#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>
#include "depthai/pipeline/datatype/ADatatype.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <filesystem>
#endif

namespace dai {

class ProtoSerializable : public ADatatypeInterface {
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
     * @brief Set from a deserialized protobuf message of this object
     */
    virtual void deserializeProto(const std::vector<std::uint8_t>& bytes) = 0;

    /**
     * @brief Serialize the schema of this object
     * @return schemaPair
     */
    virtual SchemaPair serializeSchema() const = 0;

    /**
     * @brief Serialize this object and write it to disk
     * @param path Output file path.
     * @param metadataOnly If true, serialize only metadata and omit payload data where supported by the concrete type.
     */
    void save(const std::filesystem::path& path, bool metadataOnly = false) const;

    /**
     * @brief Load this object from a serialized protobuf file on disk
     * @param path Input file path.
     */
    void load(const std::filesystem::path& path);

    /**
     * @brief Serialize the protobuf message of this object directly into a stream.
     *
     * save() goes through serializeProto(), which materializes the whole encoded
     * message in a std::vector before it is handed to the file — a full extra copy
     * of the payload. Writing straight into the stream avoids that copy, which
     * matters when several large messages are serialized concurrently.
     *
     * @param os Output stream to write the encoded message to.
     * @param metadataOnly If true, serialize only metadata and omit payload data where supported.
     * @param consume If true, the payload buffer may be released as soon as it has been
     *                copied into the protobuf message, i.e. BEFORE the (slow) encode+write.
     *                This drops the peak memory held per in-flight serialization from two
     *                copies of the payload to one, but leaves this object without its
     *                payload afterwards — only pass true if the message is dead after saving.
     *
     * The default implementation falls back to serializeProto() and honours neither
     * optimisation; concrete types override it to stream directly.
     */
    virtual void serializeProtoToStream(std::ostream& os, bool metadataOnly = false, bool consume = false);

    /**
     * @brief Serialize this object and write it to disk, streaming directly to the file.
     *
     * Byte-for-byte identical output to save(), but avoids materializing the encoded
     * message in memory first. See serializeProtoToStream() for the meaning of @p consume.
     */
    void saveStream(const std::filesystem::path& path, bool metadataOnly = false, bool consume = false);

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

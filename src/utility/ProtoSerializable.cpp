#include "depthai/utility/ProtoSerializable.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
#include "depthai/schemas/common.pb.h"
#endif

#include <fstream>

namespace dai {

ProtoSerializable::~ProtoSerializable() = default;

#ifdef DEPTHAI_ENABLE_PROTOBUF

namespace {

std::filesystem::path resolveDataPath(const std::filesystem::path& path) {
    if(path.has_extension()) {
        return path;
    }
    auto resolved = path;
    resolved += ".dai";
    return resolved;
}

void writeMsgBinaryFile(const std::filesystem::path& path, const proto::common::ProtoSerializableMessage& message) {
    std::ofstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for writing: " + path.string());
    }
    if(!message.SerializeToOstream(&file)) {
        throw std::runtime_error("Failed to write protobuf message: " + path.string());
    }
}

proto::common::ProtoSerializableMessage readMsgBinaryFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for reading: " + path.string());
    }

    proto::common::ProtoSerializableMessage message;
    if(!message.ParseFromIstream(&file)) {
        throw std::runtime_error("Failed to parse protobuf message: " + path.string());
    }
    return message;
}

}  // namespace

void ProtoSerializable::save(const std::filesystem::path& path, bool metadataOnly) const {
    proto::common::ProtoSerializableMessage message;
    message.set_schema_name(serializeSchema().schemaName);
    message.set_metadata_only(metadataOnly);

    const auto serializedMessage = serializeProto(metadataOnly);
    if(!serializedMessage.empty()) {
        message.set_proto_message(serializedMessage.data(), serializedMessage.size());
    }
    writeMsgBinaryFile(resolveDataPath(path), message);
}

void ProtoSerializable::load(const std::filesystem::path& path) {
    const auto message = readMsgBinaryFile(resolveDataPath(path));
    const auto expectedSchemaName = serializeSchema().schemaName;
    if(message.schema_name() != expectedSchemaName) {
        throw std::runtime_error("Schema mismatch when reading file: " + path.string());
    }

    const auto& serializedMessage = message.proto_message();
    deserializeProto({serializedMessage.begin(), serializedMessage.end()});
}

#endif

}  // namespace dai

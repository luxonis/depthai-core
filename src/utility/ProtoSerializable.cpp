#include "depthai/utility/ProtoSerializable.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <fstream>
    #include <stdexcept>
#endif

namespace dai {

ProtoSerializable::~ProtoSerializable() = default;

#ifdef DEPTHAI_ENABLE_PROTOBUF
namespace {

std::vector<std::uint8_t> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for reading: " + path.string());
    }

    file.seekg(0, std::ios::end);
    const auto size = file.tellg();
    if(size < 0) {
        throw std::runtime_error("Failed to determine file size: " + path.string());
    }
    file.seekg(0, std::ios::beg);

    std::vector<std::uint8_t> buffer(static_cast<size_t>(size));
    if(!buffer.empty()) {
        file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
        if(!file) {
            throw std::runtime_error("Failed to read file: " + path.string());
        }
    }
    return buffer;
}

void writeBinaryFile(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for writing: " + path.string());
    }
    if(!bytes.empty()) {
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if(!file) {
            throw std::runtime_error("Failed to write file: " + path.string());
        }
    }
}

}  // namespace

void ProtoSerializable::save(const std::filesystem::path& path, bool metadataOnly) const {
    writeBinaryFile(path, serializeProto(metadataOnly));
}

void ProtoSerializable::load(const std::filesystem::path& path, bool metadataOnly) {
    deserializeProtoMessage(readBinaryFile(path), metadataOnly);
}

void ProtoSerializable::deserializeProtoMessage(const std::vector<std::uint8_t>&, bool) {
    throw std::runtime_error("Protobuf deserialization is not implemented for this message type");
}
#endif

}  // namespace dai

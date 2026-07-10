#include "depthai/utility/ProtoSerializable.hpp"

#include <fstream>
#include <iostream>

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

void writeMsgBinaryFile(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes, DatatypeEnum datatype) {
    std::ofstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for writing: " + path.string());
    }
    if(!bytes.empty()) {
        file.write(reinterpret_cast<const char*>(&datatype), sizeof(datatype));
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if(!file) {
            throw std::runtime_error("Failed to write file: " + path.string());
        }
    }
}

std::vector<std::uint8_t> readMsgBinaryFile(const std::filesystem::path& path, DatatypeEnum datatype) {
    std::ifstream file(path, std::ios::binary);
    if(!file) {
        throw std::runtime_error("Failed to open file for reading: " + path.string());
    }

    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    if(size < 0) {
        throw std::runtime_error("Failed to determine file size: " + path.string());
    }
    if(static_cast<size_t>(size) < sizeof(datatype)) {
        throw std::runtime_error("Invalid file: " + path.string());
    }
    file.seekg(0, std::ios::beg);

    size -= sizeof(datatype);  // Subtract the size of the prepended datatype enum
    std::vector<std::uint8_t> buffer(static_cast<size_t>(size));
    if(!buffer.empty()) {
        DatatypeEnum readDatatype = DatatypeEnum::ADatatype;
        file.read(reinterpret_cast<char*>(&readDatatype), sizeof(readDatatype));
        if(readDatatype != datatype) {
            throw std::runtime_error("Datatype mismatch when reading file: " + path.string());
        }
        file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
        if(!file) {
            throw std::runtime_error("Failed to read file: " + path.string());
        }
    }
    return buffer;
}

}  // namespace

void ProtoSerializable::save(const std::filesystem::path& path, bool metadataOnly) const {
    writeMsgBinaryFile(resolveDataPath(path), serializeProto(metadataOnly), getDatatype());
}

void ProtoSerializable::load(const std::filesystem::path& path) {
    deserializeProto(readMsgBinaryFile(resolveDataPath(path), getDatatype()));
}

#endif

}  // namespace dai
